import rclpy
from rclpy.node import Node
import numpy as np
from typing import Optional

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from f110_msgs.msg import WpntArray
from frenet_conversion.frenet_converter import FrenetConverter
from tf_transformations import euler_from_quaternion, quaternion_matrix


class Carstate3D(Node):
    def __init__(self):
        super().__init__('carstate_3d')

        # Parameters
        self.declare_parameter('ekf_odom_topic', '/ekf_odom')
        self.declare_parameter('ekf_pose_topic', '/ekf_pose')
        self.declare_parameter('carstate_odom_topic', '/car_state/odom')
        self.declare_parameter('carstate_pose_topic', '/car_state/pose')
        self.declare_parameter('frenet_odom_topic', '/car_state/frenet/odom')
        self.declare_parameter('frenet_pose_topic', '/car_state/frenet/pose')

        # ### NEW: vy 주입 정책 및 필터링/경계 파라미터
        # vy_mode: 'always' = 항상 덮어쓰기, 'if_zero' = EKF vy가 0 또는 비정상일 때만, 'never' = 덮어쓰지 않음
        self.declare_parameter('vy_mode', 'always')
        self.declare_parameter('vy_ma_len', 10)          # 이동평균 길이
        self.declare_parameter('vy_max_dt', 0.2)         # dt 상한(초) - 너무 길면 스킵
        self.declare_parameter('min_speed_for_vy', 0.0005) # 아주 느릴 때 vy 계산 스킵(m/s)

        self.ekf_odom_topic = self.get_parameter('ekf_odom_topic').value
        self.ekf_pose_topic = self.get_parameter('ekf_pose_topic').value
        self.carstate_odom_topic = self.get_parameter('carstate_odom_topic').value
        self.carstate_pose_topic = self.get_parameter('carstate_pose_topic').value
        self.frenet_odom_topic = self.get_parameter('frenet_odom_topic').value
        self.frenet_pose_topic = self.get_parameter('frenet_pose_topic').value

        self.vy_mode = str(self.get_parameter('vy_mode').value).lower()
        self.vy_ma_len = int(self.get_parameter('vy_ma_len').value)
        self.vy_max_dt = float(self.get_parameter('vy_max_dt').value)
        self.min_speed_for_vy = float(self.get_parameter('min_speed_for_vy').value)

        # Subs & pubs
        self.ekf_odom: Optional[Odometry] = None
        self.ekf_pose: Optional[PoseStamped] = None
        self.frenet_converter: Optional[FrenetConverter] = None

        self.ekf_odom_sub = self.create_subscription(
            Odometry, self.ekf_odom_topic, self.ekf_odom_cb, 1
        )
        self.ekf_pose_sub = self.create_subscription(
            PoseStamped, self.ekf_pose_topic, self.ekf_pose_cb, 1
        )
        self.wpnt_sub = self.create_subscription(
            WpntArray, "/global_waypoints", self.wpnt_cb, 1
        )

        self.carstate_odom_pub = self.create_publisher(Odometry, self.carstate_odom_topic, 1)
        self.carstate_pose_pub = self.create_publisher(PoseStamped, self.carstate_pose_topic, 1)
        self.frenet_odom_pub = self.create_publisher(Odometry, self.frenet_odom_topic, 1)
        self.frenet_pose_pub = self.create_publisher(PoseStamped, self.frenet_pose_topic, 1)

        # ### NEW: 차분 기반 vy 계산을 위한 상태
        self._last_x = None
        self._last_y = None
        self._last_t = None
        self._vy_buf = np.zeros(max(1, self.vy_ma_len), dtype=float)  # 이동평균 버퍼
        self._vy_idx = 0
        self._vy_count = 0  # 버퍼에 실제로 들어간 샘플 수

        # self.timer = self.create_timer(1/80.0, self.timer_cb)
        self.timer = self.create_timer(1/40.0, self.timer_cb)
        self.get_logger().info(
            f"carstate_3d_node started. vy_mode={self.vy_mode}, vy_ma_len={self.vy_ma_len}"
        )

    def wpnt_cb(self, msg: WpntArray):
        xs = [w.x_m for w in msg.wpnts]
        ys = [w.y_m for w in msg.wpnts]
        psis = [w.psi_rad for w in msg.wpnts]
        self.frenet_converter = FrenetConverter(np.array(xs), np.array(ys), np.array(psis))

    # ---------- EKF Odom 수신 & vy 계산/주입 ----------
    def ekf_odom_cb(self, msg: Odometry):
        """
        1) EKF odom 수신
        2) pose 차분으로 vy_est 계산 (map 프레임에서 속도 → base_link 프레임으로 회전)
        3) 정책(vy_mode)에 따라 msg.twist.twist.linear.y 덮어쓰기
        4) /car_state/odom 퍼블리시 (기능 유지)
        """
        # 복사본을 만들어 헤더/프레임을 강제 세팅 (원래 동작 유지)
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = msg.child_frame_id
        odom.pose = msg.pose
        odom.twist = msg.twist

        # frame 강제
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        # --- 차분 기반 vy 계산 시작 ---
        # 현재 pose/time
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]  # yaw
        t = self._stamp_to_sec(odom.header.stamp)

        vy_est = None
        if self._last_x is not None and self._last_y is not None and self._last_t is not None:
            dt = t - self._last_t
            if dt > 0.0 and dt <= self.vy_max_dt:
                # map 프레임 속도
                vx_map = (x - self._last_x) / dt
                vy_map = (y - self._last_y) / dt

                # 너무 느릴 때는 잡음이 커지므로 스킵 가능
                speed_map = np.hypot(vx_map, vy_map)
                if speed_map >= self.min_speed_for_vy:
                    # map → base_link 회전: R^T * v_map
                    # yaw 회전행렬 사용 (quaternion_matrix로 일반화 가능)
                    # v_bl = R^T v_map  (R: base_link←map 회전)
                    c = np.cos(theta)
                    s = np.sin(theta)
                    # [vx_bl, vy_bl] = [ [ c, s], [-s, c] ] @ [vx_map, vy_map]
                    vx_bl =  c * vx_map + s * vy_map
                    vy_bl = -s * vx_map + c * vy_map

                    # 이동평균 버퍼에 넣기
                    if self.vy_ma_len > 1:
                        self._vy_buf[self._vy_idx % self.vy_ma_len] = vy_bl
                        self._vy_idx += 1
                        self._vy_count = min(self._vy_count + 1, self.vy_ma_len)
                        vy_est = float(self._vy_buf[:self._vy_count].mean())
                    else:
                        vy_est = float(vy_bl)

        # 현재 pose/time을 last로 저장
        self._last_x, self._last_y, self._last_t = x, y, t

        # --- 주입 정책 ---
        if vy_est is not None:
            ekf_vy = odom.twist.twist.linear.y
            if self.vy_mode == 'always':
                odom.twist.twist.linear.y = vy_est
            elif self.vy_mode == 'if_zero':
                if (ekf_vy is None) or (abs(ekf_vy) < 1e-6) or np.isnan(ekf_vy):
                    odom.twist.twist.linear.y = vy_est
            elif self.vy_mode == 'never':
                pass  # 아무 것도 하지 않음
            else:
                # 잘못된 모드가 들어오면 안전하게 if_zero처럼 동작
                if (ekf_vy is None) or (abs(ekf_vy) < 1e-6) or np.isnan(ekf_vy):
                    odom.twist.twist.linear.y = vy_est

        # 상태 저장 및 퍼블리시
        self.ekf_odom = odom
        self.carstate_odom_pub.publish(odom)

    def ekf_pose_cb(self, msg: PoseStamped):
        self.ekf_pose = msg
        # EKF Pose 그대로 car_state/pose 로 복사 출력 (기존 동작 유지)
        msg.header.frame_id = "map"
        self.carstate_pose_pub.publish(msg)

    def timer_cb(self):
        if self.ekf_odom is None or self.frenet_converter is None:
            return

        odom_cart = self.ekf_odom  # 여기에 이미 vy 주입이 반영되어 있을 수 있음
        x = odom_cart.pose.pose.position.x
        y = odom_cart.pose.pose.position.y
        vx = odom_cart.twist.twist.linear.x
        vy = odom_cart.twist.twist.linear.y
        q = odom_cart.pose.pose.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # Frenet 변환 (기존 로직 유지, 단 vy는 주입된 값을 사용)
        frenet_pos = self.frenet_converter.get_frenet([x], [y])
        frenet_vel = self.frenet_converter.get_frenet_velocities(vx, vy, theta)

        s, d = frenet_pos[0, 0], frenet_pos[1, 0]
        vs, vd = frenet_vel[0][0], frenet_vel[1][0]

        # Pose
        frenet_pose_msg = PoseStamped()
        frenet_pose_msg.header = odom_cart.header
        frenet_pose_msg.header.frame_id = "frenet"
        frenet_pose_msg.pose.position.x = s
        frenet_pose_msg.pose.position.y = d

        # Odom
        frenet_odom_msg = Odometry()
        frenet_odom_msg.header = frenet_pose_msg.header
        frenet_odom_msg.pose.pose = frenet_pose_msg.pose
        frenet_odom_msg.twist.twist.linear.x = vs
        frenet_odom_msg.twist.twist.linear.y = vd

        # Publish
        self.frenet_pose_pub.publish(frenet_pose_msg)
        self.frenet_odom_pub.publish(frenet_odom_msg)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        """builtin_interfaces/Time → seconds(float)"""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main():
    rclpy.init()
    node = Carstate3D()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
