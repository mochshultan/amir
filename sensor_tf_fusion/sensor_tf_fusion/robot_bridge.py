#!/usr/bin/env python3
"""
robot_bridge.py
===============
Node ROS 2 tunggal yang menggabungkan fungsi dari:
  - laser_static_tf.py      → static TF base_link → laser (opsional)
  - laser_tf_bridge.py      → dynamic TF base_link → laser @10 Hz
  - scan_timestamp_fix.py   → sinkronisasi timestamp /scan → /scan_fixed
  - sensor_fusion_bridge.py → odometri + TF odom → base_link + republish scan

Cara pakai:
  ros2 run <package> robot_bridge
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

try:
    import tf_transformations
    USE_TF_TRANSFORMATIONS = True
except ImportError:
    USE_TF_TRANSFORMATIONS = False


# ===========================================================================
# Helper: Euler → Quaternion (fallback jika tf_transformations tidak ada)
# ===========================================================================
def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Konversi sudut Euler (rad) ke Quaternion [x, y, z, w]."""
    if USE_TF_TRANSFORMATIONS:
        return tf_transformations.quaternion_from_euler(roll, pitch, yaw)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    ]


# ===========================================================================
# Node Utama
# ===========================================================================
class RobotBridge(Node):
    """
    Satu node yang menangani:
      1. Static TF  : base_link → laser (dikirim sekali saat startup)
      2. Dynamic TF : base_link → laser @10 Hz  (opsional, aktif via param)
      3. Scan fix   : /scan → /scan_fixed (timestamp diganti clock ROS)
      4. Odometry   : odom_data → /odom + TF odom → base_link @10 Hz
    """

    def __init__(self):
        super().__init__('robot_bridge')

        # ------------------------------------------------------------------
        # Parameter (bisa di-override dari launch file / command line)
        # ------------------------------------------------------------------
        self.declare_parameter('use_static_tf',    True)   # kirim static TF sekali
        self.declare_parameter('use_dynamic_laser_tf', False)  # TF laser dinamis @10Hz

        # Posisi lidar relatif ke base_link
        self.declare_parameter('laser_x',   0.22)
        self.declare_parameter('laser_y',   0.165)
        self.declare_parameter('laser_z',   0.165)
        self.declare_parameter('laser_yaw', math.pi)   # 180°

        # Frame ID
        self.declare_parameter('base_frame',  'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('odom_frame',  'odom')

        # Rate publish
        self.declare_parameter('publish_rate_hz', 10.0)

        # Baca parameter
        self.use_static_tf       = self.get_parameter('use_static_tf').value
        self.use_dynamic_laser   = self.get_parameter('use_dynamic_laser_tf').value
        self.laser_x             = self.get_parameter('laser_x').value
        self.laser_y             = self.get_parameter('laser_y').value
        self.laser_z             = self.get_parameter('laser_z').value
        self.laser_yaw           = self.get_parameter('laser_yaw').value
        self.base_frame          = self.get_parameter('base_frame').value
        self.laser_frame         = self.get_parameter('laser_frame').value
        self.odom_frame          = self.get_parameter('odom_frame').value
        rate_hz                  = self.get_parameter('publish_rate_hz').value

        # ------------------------------------------------------------------
        # State odometri
        # ------------------------------------------------------------------
        self.x              = 0.0
        self.y              = 0.0
        self.th             = 0.0
        self.last_odom_stamp = None

        # Rate-limiter untuk republish scan
        self.min_interval_ns = int(1e9 / rate_hz)
        self.last_scan_pub_ns = 0

        # ------------------------------------------------------------------
        # Broadcaster
        # ------------------------------------------------------------------
        self.tf_broadcaster        = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry,   '/odom',       10)
        self.scan_pub = self.create_publisher(LaserScan,  '/scan_fixed', 10)

        # ------------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------------
        self.create_subscription(Float32MultiArray, 'odom_data',  self.odom_callback, 10)
        self.create_subscription(LaserScan,         '/scan',      self.scan_callback, 10)

        # ------------------------------------------------------------------
        # Timer utama
        # ------------------------------------------------------------------
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_odom_and_tf)

        # ------------------------------------------------------------------
        # Static TF: base_link → laser (dikirim sekali saat startup)
        # ------------------------------------------------------------------
        if self.use_static_tf:
            self._send_static_laser_tf()

        self.get_logger().info(
            f"✅ RobotBridge aktif | rate={rate_hz} Hz | "
            f"static_tf={self.use_static_tf} | dynamic_laser_tf={self.use_dynamic_laser}"
        )

    # -----------------------------------------------------------------------
    # 1. Static TF: base_link → laser
    # -----------------------------------------------------------------------
    def _send_static_laser_tf(self):
        t = TransformStamped()
        t.header.stamp.sec     = 0
        t.header.stamp.nanosec = 0
        t.header.frame_id      = self.base_frame
        t.child_frame_id       = self.laser_frame

        t.transform.translation.x = self.laser_x
        t.transform.translation.y = self.laser_y
        t.transform.translation.z = self.laser_z

        q = euler_to_quaternion(0.0, 0.0, self.laser_yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"📡 Static TF dikirim: {self.base_frame} → {self.laser_frame} (stamp=0)"
        )

    # -----------------------------------------------------------------------
    # 2. Dynamic TF: base_link → laser (dipanggil dari timer jika aktif)
    # -----------------------------------------------------------------------
    def _publish_dynamic_laser_tf(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id  = self.laser_frame

        t.transform.translation.x = self.laser_x
        t.transform.translation.y = self.laser_y
        t.transform.translation.z = self.laser_z

        q = euler_to_quaternion(0.0, 0.0, self.laser_yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    # -----------------------------------------------------------------------
    # 3. Scan callback: /scan → /scan_fixed (timestamp fix + rate limit)
    # -----------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()

        # Rate limiter
        if (now.nanoseconds - self.last_scan_pub_ns) < self.min_interval_ns:
            return

        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = self.laser_frame
        self.scan_pub.publish(msg)
        self.last_scan_pub_ns = now.nanoseconds

    # -----------------------------------------------------------------------
    # 4. Odom callback: data mentah dari mikrokontroler
    # -----------------------------------------------------------------------
    def odom_callback(self, msg: Float32MultiArray):
        try:
            x_raw, y_raw, th_raw = msg.data

            # Koreksi orientasi (offset 180°)
            th = th_raw + math.pi
            if th > math.pi:
                th -= 2 * math.pi

            self.x              = -x_raw
            self.y              = -y_raw
            self.th             = th
            self.last_odom_stamp = self.get_clock().now().to_msg()

        except Exception as e:
            self.get_logger().warn(f"⚠️  Odom data error: {e}")

    # -----------------------------------------------------------------------
    # 5. Timer: publish /odom + TF odom→base_link (+ laser TF jika dinamis)
    # -----------------------------------------------------------------------
    def publish_odom_and_tf(self):
        now = self.get_clock().now().to_msg()

        # TF laser dinamis (jika diaktifkan)
        if self.use_dynamic_laser:
            self._publish_dynamic_laser_tf()

        # Belum ada data odometri → skip
        if self.last_odom_stamp is None:
            return

        q = euler_to_quaternion(0.0, 0.0, self.th)

        # --- Odometry message ---
        odom = Odometry()
        odom.header.stamp        = now
        odom.header.frame_id     = self.odom_frame
        odom.child_frame_id      = self.base_frame
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)

        # --- TF: odom → base_link ---
        t = TransformStamped()
        t.header.stamp        = now
        t.header.frame_id     = self.odom_frame
        t.child_frame_id      = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


# ===========================================================================
# Entry point
# ===========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = RobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
