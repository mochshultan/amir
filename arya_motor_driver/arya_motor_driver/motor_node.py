#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from .zlac_driver import ZLACDriver, i16, i32

# ZLAC8015D Register Addresses
REG_POS_L_HI = 0x20A7
REG_POS_R_HI = 0x20A9
REG_BUS_VOLT = 0x20A1
REG_ACT_TORQUE_L = 0x20AD
REG_ACT_TORQUE_R = 0x20AE
REG_DRIVER_TEMP = 0x20B0


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # === Declare all parameters ===
        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('wheel_radius', 0.1016)
        self.declare_parameter('wheel_separation', 0.293)
        self.declare_parameter('counts_per_rev', 16384)
        self.declare_parameter('invert_left', True)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('enable_on_startup', True)
        self.declare_parameter('max_rpm', 300.0)
        self.declare_parameter('accel_rate_rpm_per_s', 500.0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 1.0)

        # === Load parameter values ===
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baudrate').value)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_separation').value)
        self.cpr = int(self.get_parameter('counts_per_rev').value)
        legacy_inv_l = bool(self.get_parameter('invert_left').value)
        legacy_inv_r = bool(self.get_parameter('invert_right').value)

        # Keep backward compatibility with old invert_left/right while allowing
        # command and encoder sign conventions to be tuned independently.
        self.declare_parameter('invert_left_cmd', legacy_inv_l)
        self.declare_parameter('invert_right_cmd', legacy_inv_r)
        self.declare_parameter('invert_left_odom', legacy_inv_l)
        self.declare_parameter('invert_right_odom', legacy_inv_r)

        self.inv_l_cmd = bool(self.get_parameter('invert_left_cmd').value)
        self.inv_r_cmd = bool(self.get_parameter('invert_right_cmd').value)
        self.inv_l_odom = bool(self.get_parameter('invert_left_odom').value)
        self.inv_r_odom = bool(self.get_parameter('invert_right_odom').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.accel_rate = float(self.get_parameter('accel_rate_rpm_per_s').value)
        self.enable_on_startup = bool(self.get_parameter('enable_on_startup').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)

        # === Driver setup ===
        self.driver = ZLACDriver(port=self.port, baudrate=self.baud)
        if self.enable_on_startup:
            self.driver.enable()
            self.get_logger().info(f"✅ Motor enabled on {self.port} @ {self.baud} bps")

        # === Motion & feedback state ===
        self.last_cmd_time = time.time()
        self.current_rpm_l = 0.0
        self.current_rpm_r = 0.0
        self.target_rpm_l = 0.0
        self.target_rpm_r = 0.0
        self.x = self.y = self.th = 0.0
        self.last_L = self.last_R = None

        # === ROS publishers/subscribers ===
        qos = QoSProfile(depth=10)
        self.pub_odom = self.create_publisher(Float32MultiArray, 'odom_data', qos)
        self.pub_tele = self.create_publisher(Float32MultiArray, 'telemetry', qos)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cb_cmd_vel, qos)
        self.sub_reset_odom = self.create_subscription(Float32MultiArray, 'reset_odom', self.cb_reset_odom, qos)
        self.sub_reset_enc = self.create_subscription(Float32MultiArray, 'reset_encoder', self.cb_reset_encoder, qos)

        # === Timers ===
        self.create_timer(1.0 / self.publish_rate, self.update_feedback)
        self.create_timer(0.05, self.control_loop)
        self.last_debug = time.time()

        self.get_logger().info(
            f"[MotorNode] aktif | r={self.r:.3f} | L={self.L:.3f} | CPR={self.cpr} | "
            f"inv_cmd(L/R)={self.inv_l_cmd}/{self.inv_r_cmd} | "
            f"inv_odom(L/R)={self.inv_l_odom}/{self.inv_r_odom}"
        )

    # === Velocity command callback ===
    def cb_cmd_vel(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        v_l = (v - w * self.L / 2.0) / self.r
        v_r = (v + w * self.L / 2.0) / self.r
        rpm_l = v_l * 60 / (2 * math.pi)
        rpm_r = v_r * 60 / (2 * math.pi)

        if self.inv_l_cmd: rpm_l = -rpm_l
        if self.inv_r_cmd: rpm_r = -rpm_r

        # Clamp ke max_rpm
        rpm_l = max(min(rpm_l, self.max_rpm), -self.max_rpm)
        rpm_r = max(min(rpm_r, self.max_rpm), -self.max_rpm)

        self.target_rpm_l = rpm_l
        self.target_rpm_r = rpm_r
        self.last_cmd_time = time.time()

    # === Smooth acceleration control ===
    def control_loop(self):
        now = time.time()

        # Auto-stop jika timeout
        if now - self.last_cmd_time > self.cmd_timeout:
            self.target_rpm_l = 0.0
            self.target_rpm_r = 0.0

        # Akselerasi halus ke target
        step = self.accel_rate * 0.05  # 50ms loop
        def smooth(curr, target):
            if abs(target - curr) < step:
                return target
            return curr + step if target > curr else curr - step

        self.current_rpm_l = smooth(self.current_rpm_l, self.target_rpm_l)
        self.current_rpm_r = smooth(self.current_rpm_r, self.target_rpm_r)

        self.driver.set_speed(self.current_rpm_l, self.current_rpm_r)

    # === Reset callback ===
    def cb_reset_odom(self, msg):
        self.x = self.y = self.th = 0.0
        self.last_L = self.last_R = None
        self.get_logger().info("🧭 Odometry direset ke (0,0,0)")

    def cb_reset_encoder(self, msg):
        self.driver.stop()
        self.get_logger().info("⚙️ Encoder / motor RPM direset ke nol")

    # === Feedback loop ===
    def update_feedback(self):
        try:
            pos_L = self.driver.read_registers(REG_POS_L_HI, 2)
            pos_R = self.driver.read_registers(REG_POS_R_HI, 2)
            if not pos_L or not pos_R:
                return

            enc_L = i32(pos_L[0], pos_L[1])
            enc_R = i32(pos_R[0], pos_R[1])
            if self.last_L is None:
                self.last_L, self.last_R = enc_L, enc_R
                return

            dL = enc_L - self.last_L
            dR = enc_R - self.last_R
            self.last_L, self.last_R = enc_L, enc_R
            if self.inv_l_odom: dL = -dL
            if self.inv_r_odom: dR = -dR

            # Odom calculation
            dist_L = (dL / self.cpr) * (2 * math.pi * self.r)
            dist_R = (dR / self.cpr) * (2 * math.pi * self.r)
            dS = (dist_R + dist_L) / 2
            dTh = (dist_R - dist_L) / self.L
            th_mid = self.th + 0.5 * dTh
            self.x += dS * math.cos(th_mid)
            self.y += dS * math.sin(th_mid)
            self.th = self.normalize_angle(self.th + dTh)

            # Publish odom
            odom = Float32MultiArray()
            odom.data = [self.x, self.y, self.th]
            self.pub_odom.publish(odom)

            # Telemetry
            vbus = self.driver.read_u16(REG_BUS_VOLT) or 0
            torqL = self.driver.read_u16(REG_ACT_TORQUE_L) or 0
            torqR = self.driver.read_u16(REG_ACT_TORQUE_R) or 0
            temp = self.driver.read_u16(REG_DRIVER_TEMP) or 0
            rpmL, rpmR = self.driver.read_actual_velocity() or (0, 0)

            tele = Float32MultiArray()
            tele.data = [
                float(vbus) * 0.01,
                float(i16(torqL)) * 0.1,
                float(i16(torqR)) * 0.1,
                float(i16(temp)) * 0.1,
                float(rpmL),
                float(rpmR)
            ]
            self.pub_tele.publish(tele)

            now = time.time()
            if now - self.last_debug > 1.0:
                self.get_logger().info(
                    f"🔋 {tele.data[0]:.1f}V | Temp {tele.data[3]:.1f}°C | "
                    f"RPM L/R={tele.data[4]:.0f}/{tele.data[5]:.0f} | "
                    f"Pose X={self.x:.2f}, Y={self.y:.2f}, Th={math.degrees(self.th):.1f}°"
                )
                self.last_debug = now
        except Exception as e:
            self.get_logger().warn(f"Loop feedback gagal: {e}")

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.stop()
        node.driver.disable()
        node.driver.close()
        node.destroy_node()
        rclpy.shutdown()
        print("🛑 MotorNode shutdown complete.")


if __name__ == "__main__":
    main()

