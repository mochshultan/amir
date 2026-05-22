#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster


# ──────────────────────────────────────────────
# Covariance diagonal values (tune as needed)
# ──────────────────────────────────────────────
# Pose: [x, y, z, roll, pitch, yaw]
POSE_COV_X      = 0.02   # x position noise (m²)
POSE_COV_Y      = 0.02  # y position noise (m²)
POSE_COV_Z      = 0.1    # z (unused in 2D, keep non-zero)
POSE_COV_ROLL   = 0.1    # roll (unused in 2D)
POSE_COV_PITCH  = 0.1    # pitch (unused in 2D)
POSE_COV_YAW    = 0.03   # yaw noise (rad²)

# Twist: [vx, vy, vz, wx, wy, wz]
TWIST_COV_VX    = 0.01   # linear x velocity noise
TWIST_COV_VY    = 0.02   # linear y velocity noise
TWIST_COV_VZ    = 0.1
TWIST_COV_WX    = 0.1
TWIST_COV_WY    = 0.1
TWIST_COV_WZ    = 0.01

def make_diagonal_covariance(d0, d1, d2, d3, d4, d5):
    """Build a 6x6 diagonal covariance matrix (row-major, 36 elements)."""
    cov = [0.0] * 36
    cov[0]  = d0
    cov[7]  = d1
    cov[14] = d2
    cov[21] = d3
    cov[28] = d4
    cov[35] = d5
    return cov


POSE_COV = make_diagonal_covariance(
    POSE_COV_X, POSE_COV_Y, POSE_COV_Z,
    POSE_COV_ROLL, POSE_COV_PITCH, POSE_COV_YAW
)

TWIST_COV = make_diagonal_covariance(
    TWIST_COV_VX, TWIST_COV_VY, TWIST_COV_VZ,
    TWIST_COV_WX, TWIST_COV_WY, TWIST_COV_WZ
)


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('yaw_offset', 0.0)
        self.declare_parameter('wheel_radius', 0.1016)
        self.declare_parameter('wheel_separation', 0.293)
        self.declare_parameter('invert_left_odom', True)
        self.declare_parameter('invert_right_odom', False)
        self.declare_parameter('telemetry_timeout_sec', 0.5)

        self.invert_x = bool(self.get_parameter('invert_x').value)
        self.invert_y = bool(self.get_parameter('invert_y').value)
        self.yaw_offset = float(self.get_parameter('yaw_offset').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.invert_left_odom = bool(self.get_parameter('invert_left_odom').value)
        self.invert_right_odom = bool(self.get_parameter('invert_right_odom').value)
        self.telemetry_timeout = float(self.get_parameter('telemetry_timeout_sec').value)

        # === Variabel penyimpanan terakhir ===
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Velocity (needed for twist)
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.last_telemetry_time = None

        # === Subscriber ke data encoder ===
        self.sub_odom = self.create_subscription(
            Float32MultiArray,
            'odom_data',
            self.odom_callback,
            50
        )
        self.sub_telemetry = self.create_subscription(
            Float32MultiArray,
            'telemetry',
            self.telemetry_callback,
            50
        )

        # === Publisher odometry ===
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)

        # === TF broadcaster ===
        self.tf_broadcaster = TransformBroadcaster(self)

        # === Timer publish setiap 0.05 s (20 Hz) === 0.033 (30hz)
        self.timer = self.create_timer(0.033, self.publish_odom)

        self.publish_count = 0
        self.get_logger().info("✅ OdomBridge aktif — mempublikasikan /odom dan /tf @30 Hz")

    # -------------------------------------------------
    # Callback data encoder dari microcontroller
    # -------------------------------------------------
    def odom_callback(self, msg):
        try:
            x_raw, y_raw, th_raw = msg.data

            # ROS base_link convention: +X forward, +Y left, +Z up.
            # Keep odometry in that convention; use params only for hardware-specific corrections.
            th = self.normalize_angle(th_raw + self.yaw_offset)
            x_adj = -x_raw if self.invert_x else x_raw
            y_adj = -y_raw if self.invert_y else y_raw

            # Keep pose frame consistent: yaw offset must rotate x/y too,
            # not only heading, otherwise left/right semantics become wrong.
            if abs(self.yaw_offset) > 1e-12:
                c = math.cos(self.yaw_offset)
                s = math.sin(self.yaw_offset)
                self.x = c * x_adj - s * y_adj
                self.y = s * x_adj + c * y_adj
            else:
                self.x = x_adj
                self.y = y_adj
            self.th = th

        except Exception as e:
            self.get_logger().warn(f"⚠️ Format odom_data tidak sesuai: {e}")
            return

    def telemetry_callback(self, msg):
        try:
            if len(msg.data) < 6:
                raise ValueError("telemetry butuh >= 6 elemen")

            rpm_l = float(msg.data[4])
            rpm_r = float(msg.data[5])

            # Match odometry sign convention from motor node configuration.
            if self.invert_left_odom:
                rpm_l = -rpm_l
            if self.invert_right_odom:
                rpm_r = -rpm_r

            omega_l = rpm_l * (2.0 * math.pi / 60.0)
            omega_r = rpm_r * (2.0 * math.pi / 60.0)
            v_l = omega_l * self.wheel_radius
            v_r = omega_r * self.wheel_radius

            self.vx = 0.5 * (v_r + v_l)
            self.vy = 0.0
            if abs(self.wheel_separation) > 1e-9:
                self.vth = (v_r - v_l) / self.wheel_separation
            else:
                self.vth = 0.0

            self.last_telemetry_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().warn(f"⚠️ Format telemetry tidak sesuai: {e}")

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # -------------------------------------------------
    # Publikasi Odometry + Transform TF
    # -------------------------------------------------
    def publish_odom(self):
        now_clock = self.get_clock().now()
        now = now_clock.to_msg()

        # Prevent stale telemetry from being published as current velocity.
        if self.last_telemetry_time is None:
            self.vx = self.vy = self.vth = 0.0
        else:
            dt_telemetry = (now_clock - self.last_telemetry_time).nanoseconds * 1e-9
            if dt_telemetry > self.telemetry_timeout:
                self.vx = self.vy = self.vth = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.th)

        # === Publikasi Odometry ===
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance = POSE_COV          # ← FIX: was all zeros

        # Twist
        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.linear.y  = self.vy
        odom.twist.twist.angular.z = self.vth
        odom.twist.covariance = TWIST_COV        # ← FIX: was all zeros

        self.odom_pub.publish(odom)

        # === Publikasi TF ===
        # t = TransformStamped()
        # t.header.stamp = now
        # t.header.frame_id = "odom"
        # t.child_frame_id = "base_link"
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # self.tf_broadcaster.sendTransform(t)

        self.publish_count += 1
        if self.publish_count % 30 == 0:
            self.get_logger().info(
                f"📡 Odom — X:{self.x:.3f} Y:{self.y:.3f} "
                f"θ:{math.degrees(self.th):.1f}° "
                f"vx:{self.vx:.3f} vth:{math.degrees(self.vth):.1f}°/s"
            )


# -------------------------------------------------
# Main
# -------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
