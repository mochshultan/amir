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
        self.declare_parameter('wheel_radius', 0.1015) #1016
        self.declare_parameter('wheel_separation', 0.293)
        self.declare_parameter('counts_per_rev', 16384)
        self.declare_parameter('invert_left', True)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('enable_on_startup', True)
        self.declare_parameter('max_rpm', 300.0)
        self.declare_parameter('accel_rate_rpm_per_s', 500.0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 1.0)
        self.declare_parameter('deadband_compensation_enabled', False)
        self.declare_parameter('linear_min_cmd_mps', 0.0)
        self.declare_parameter('angular_min_cmd_radps', 0.0)
        self.declare_parameter('linear_zero_epsilon_mps', 0.0)
        self.declare_parameter('angular_zero_epsilon_radps', 0.0)

        # === Load parameter values ===
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baudrate').value)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.bias = float(-0.000)
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
        self.publish_rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.control_rate = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self.deadband_comp_enabled = bool(self.get_parameter('deadband_compensation_enabled').value)
        self.linear_min_cmd = max(0.0, float(self.get_parameter('linear_min_cmd_mps').value))
        self.angular_min_cmd = max(0.0, float(self.get_parameter('angular_min_cmd_radps').value))
        self.linear_zero_epsilon = max(0.0, float(self.get_parameter('linear_zero_epsilon_mps').value))
        self.angular_zero_epsilon = max(0.0, float(self.get_parameter('angular_zero_epsilon_radps').value))

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
        self.last_vbus = 0.0
        self.last_vbus_time = 0.0

        # === ROS publishers/subscribers ===
        qos = QoSProfile(depth=10)
        self.pub_odom = self.create_publisher(Float32MultiArray, 'odom_data', qos)
        self.pub_tele = self.create_publisher(Float32MultiArray, 'telemetry', qos)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cb_cmd_vel, qos)
        self.sub_reset_odom = self.create_subscription(Float32MultiArray, 'reset_odom', self.cb_reset_odom, qos)
        self.sub_reset_enc = self.create_subscription(Float32MultiArray, 'reset_encoder', self.cb_reset_encoder, qos)

        # === Timers ===
        self.create_timer(1.0 / self.publish_rate, self.update_feedback)
        self.create_timer(1.0 / self.control_rate, self.control_loop)
        self.last_debug = time.time()

        self.get_logger().info(
            f"[MotorNode] aktif | r={self.r:.3f} | L={self.L:.3f} | CPR={self.cpr} | "
            f"inv_cmd(L/R)={self.inv_l_cmd}/{self.inv_r_cmd} | "
            f"inv_odom(L/R)={self.inv_l_odom}/{self.inv_r_odom}"
        )
        self.get_logger().info(
            f"[MotorNode] deadband compensation={self.deadband_comp_enabled} | "
            f"linear_min={self.linear_min_cmd:.3f} m/s | angular_min={self.angular_min_cmd:.3f} rad/s | "
            f"eps linear/angular={self.linear_zero_epsilon:.3f}/{self.angular_zero_epsilon:.3f}"
        )
        self.get_logger().info(
            f"[MotorNode] rates | control={self.control_rate:.1f} Hz | feedback={self.publish_rate:.1f} Hz"
        )

    # === Velocity command callback ===
    def cb_cmd_vel(self, msg: Twist):
        v = self.compensate_deadband(
            msg.linear.x,
            self.linear_min_cmd,
            self.linear_zero_epsilon
        )
        w = self.compensate_deadband(
            msg.angular.z,
            self.angular_min_cmd,
            self.angular_zero_epsilon
        )
        v_l = (v - w * self.L / 2.0) / (self.r-self.bias)
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
        step = self.accel_rate / self.control_rate
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
            feedback = self.driver.read_feedback_core()
            if feedback is None:
                return

            enc_L = i32(feedback["enc_l_hi"], feedback["enc_l_lo"])
            enc_R = i32(feedback["enc_r_hi"], feedback["enc_r_lo"])
            if self.last_L is None:
                self.last_L, self.last_R = enc_L, enc_R
                return

            dL = enc_L - self.last_L
            dR = enc_R - self.last_R
            self.last_L, self.last_R = enc_L, enc_R
            if self.inv_l_odom: dL = -dL
            if self.inv_r_odom: dR = -dR

            # Odom calculation
            dist_L = (dL / self.cpr) * (2 * math.pi * (self.r - self.bias))
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
            now = time.time()
            if now - self.last_vbus_time > 1.0:
                self.last_vbus = float(self.driver.read_u16(REG_BUS_VOLT) or 0) * 0.01
                self.last_vbus_time = now

            torqL = i16(feedback["torq_l_raw"])
            torqR = i16(feedback["torq_r_raw"])
            temp = i16(feedback["temp_raw"])
            rpmL = i16(feedback["rpm_l_raw"]) / 10.0
            rpmR = i16(feedback["rpm_r_raw"]) / 10.0

            tele = Float32MultiArray()
            tele.data = [
                self.last_vbus,
                float(torqL) * 0.1,
                float(torqR) * 0.1,
                float(temp) * 0.1,
                float(rpmL),
                float(rpmR)
            ]
            self.pub_tele.publish(tele)

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

    def compensate_deadband(self, value, minimum, zero_epsilon):
        if not self.deadband_comp_enabled:
            return value
        if abs(value) <= zero_epsilon:
            return 0.0
        if minimum <= 0.0:
            return value
        if abs(value) < minimum:
            return math.copysign(minimum, value)
        return value


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
