import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, String, Bool
from waveshare_modbus_io_interfaces.srv import SetSingle, SetMask
from .modbus_client import WaveshareIOClient
import time

def bits_to_byte(bits8):
    b = 0
    for i, v in enumerate(bits8):
        if v:
            b |= (1 << i)
    return b

def names_from_mask(mask, names):
    return ",".join([names[i] for i in range(8) if (mask >> i) & 1])

class WaveshareIONode(Node):
    def __init__(self):
        super().__init__("waveshare_modbus_io")

        # --- parameters ---
        self.declare_parameter("port", "/dev/ttyS3")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("parity", "N")
        self.declare_parameter("stopbits", 1)
        self.declare_parameter("bytesize", 8)
        self.declare_parameter("slave_id", 1)
        self.declare_parameter("poll_rate_hz", 20.0)
        self.declare_parameter("publish_outputs", True)
        self.declare_parameter("debounce_ms", 20)
        self.declare_parameter("reconnect_sec", 2.0)

        # mapping nama DI/DO (bisa di-override dari YAML)
        self.declare_parameter("di_names", [f"DI{i+1}" for i in range(8)])
        self.declare_parameter("do_names", [f"DO{i+1}" for i in range(8)])

        # GANTI cfg menjadi:
        cfg = dict(
            # port=self.get_parameter("port").get_parameter_value().string_value,
            port="/dev/ttyS3",
            baudrate=self.get_parameter("baudrate").get_parameter_value().integer_value,
            parity=self.get_parameter("parity").get_parameter_value().string_value,
            stopbits=self.get_parameter("stopbits").get_parameter_value().integer_value,
            bytesize=self.get_parameter("bytesize").get_parameter_value().integer_value,
            unit=self.get_parameter("slave_id").get_parameter_value().integer_value,
            reconnect_sec=self.get_parameter("reconnect_sec").get_parameter_value().double_value,
        )

        self.client = WaveshareIOClient(**cfg)
        self.poll_dt = 1.0 / self.get_parameter("poll_rate_hz").get_parameter_value().double_value
        self.publish_outputs = self.get_parameter("publish_outputs").get_parameter_value().bool_value
        self.debounce_ms = self.get_parameter("debounce_ms").get_parameter_value().integer_value
        self.di_names = list(self.get_parameter("di_names").get_parameter_value().string_array_value)
        self.do_names = list(self.get_parameter("do_names").get_parameter_value().string_array_value)

        # --- publishers (byte/bit & event) ---
        self.pub_inputs = self.create_publisher(UInt8, "io/inputs_raw", 10)
        self.pub_outputs = self.create_publisher(UInt8, "io/outputs_raw", 10) if self.publish_outputs else None
        self.pub_di_rising = self.create_publisher(UInt8, "io/di_rising", 10)
        self.pub_di_falling = self.create_publisher(UInt8, "io/di_falling", 10)
        self.pub_di_rising_names = self.create_publisher(String, "io/di_rising_names", 10)
        self.pub_di_falling_names = self.create_publisher(String, "io/di_falling_names", 10)

        # per-bit publishers
        #self.di_bit_pubs = [self.create_publisher(Bool, f"io/di/{i+1}", 10) for i in range(8)]
        #self.do_bit_pubs = [self.create_publisher(Bool, f"io/do/{i+1}", 10) for i in range(8)] if self.publish_outputs else []
       
        self.di_bit_pubs = [self.create_publisher(Bool, f"io/di{i+1}", 10) for i in range(8)]
        self.do_bit_pubs = [self.create_publisher(Bool, f"io/do{i+1}", 10) for i in range(8)]

        # --- subscriptions (legacy/kompat) ---
        self.sub_cmd_single = self.create_subscription(String, "io/cmd_single", self.on_cmd_single, 10)
        self.sub_cmd_mask = self.create_subscription(UInt8, "io/cmd_mask", self.on_cmd_mask, 10)

        # --- services ---
        self.srv_set_single = self.create_service(SetSingle, "io/set_single", self.handle_set_single)
        self.srv_set_mask = self.create_service(SetMask, "io/set_mask", self.handle_set_mask)

        # --- state ---
        self.last_inputs_byte = None
        self.prev_inputs_byte = 0
        self.prev_outputs_byte = 0
        self.last_change_time = time.time()

        # --- timer ---
        self.timer = self.create_timer(self.poll_dt, self.loop)
        self.get_logger().info("Waveshare Modbus IO node started")
        self.get_logger().info(
            f"port={cfg['port']}, baud={cfg['baudrate']}, parity={cfg['parity']}, "
            f"stopbits={cfg['stopbits']}, bytesize={cfg['bytesize']}, slave_id={cfg['unit']}"
        )
        try:
            self.client.write_single(4, "on")  # DO4 = HIGH
            self.get_logger().info("Default DO4 set to HIGH")
        except Exception as e:
            self.get_logger().error(f"Gagal set default DO4: {e}")

    # ----- Service handlers -----
    def handle_set_single(self, req: SetSingle.Request, res: SetSingle.Response):
        try:
            ch = int(req.channel)
            cmd_raw = str(req.cmd).strip()
            cmd = cmd_raw.lower()

            # Normalisasi: terima on/off/toggle & variasinya (true/false, 1/0, yes/no)
            if cmd in ("on", "true", "1", "yes", "y"):
                cmd_norm = "on"
            elif cmd in ("off", "false", "0", "no", "n"):
                cmd_norm = "off"
            elif cmd in ("toggle", "tgl", "tg", "switch"):
                cmd_norm = "toggle"
            else:
                raise ValueError(f"cmd must be on/off/toggle (got: {cmd_raw})")

            if not (1 <= ch <= 8):
                raise ValueError("channel must be 1..8")

            self.client.write_single(ch, cmd_norm)
            res.success = True
            res.message = f"DO{ch} <- {cmd_norm}"
            self.get_logger().info(res.message)
        except Exception as e:
            res.success = False
            res.message = f"set_single error: {e}"
            self.get_logger().error(res.message)
        return res


    def handle_set_mask(self, req: SetMask.Request, res: SetMask.Response):
        try:
            mask = int(req.mask) & 0xFF
            self.client.write_mask(mask)
            res.success = True
            res.message = f"DO mask <- 0x{mask:02X}"
            self.get_logger().info(res.message)
        except Exception as e:
            res.success = False
            res.message = f"set_mask error: {e}"
            self.get_logger().error(res.message)
        return res
       
    # def handle_set_mask(self, req: SetMask.Request, res: SetMask.Response):
    #     try:
    #         mask = int(req.mask) & 0xFF
    #         # XOR dengan state saat ini = toggle bit yang diminta
    #         new_mask = (self.prev_outputs_byte ^ mask) & 0xFF
    #         self.client.write_mask(new_mask)
    #         res.success = True
    #         res.message = f"DO mask toggle 0x{mask:02X} -> state now 0x{new_mask:02X}"
    #         self.get_logger().info(res.message)
    #     except Exception as e:
    #         res.success = False
    #         res.message = f"set_mask error: {e}"
    #         self.get_logger().error(res.message)
    #     return res

    # ----- Topic handlers (legacy/kompat) -----
    def on_cmd_single(self, msg: String):
        try:
            parts = msg.data.strip().split()
            if len(parts) != 2:
                raise ValueError("format harus: '<on|off|toggle> <channel(1..8)>'")
            cmd = parts[0].lower()
            ch = int(parts[1])
            self.client.write_single(ch, cmd)
            self.get_logger().info(f"DO{ch} <- {cmd}")
        except Exception as e:
            self.get_logger().error(f"cmd_single error: {e}")

    def on_cmd_mask(self, msg: UInt8):
        try:
            self.client.write_mask(msg.data)
            self.get_logger().info(f"DO mask <- 0x{msg.data:02X}")
        except Exception as e:
            self.get_logger().error(f"cmd_mask error: {e}")

    def _publish_di_bits_changed(self, prev_byte, cur_byte):
        changed = (prev_byte ^ cur_byte) & 0xFF
        if changed == 0:
            return
        for i in range(8):
            if (changed >> i) & 1:
                self.di_bit_pubs[i].publish(Bool(data=bool((cur_byte >> i) & 1)))

    def _publish_do_bits_changed(self, prev_byte, cur_byte):
        changed = (prev_byte ^ cur_byte) & 0xFF
        if changed == 0 or not self.do_bit_pubs:
            return
        for i in range(8):
            if (changed >> i) & 1:
                self.do_bit_pubs[i].publish(Bool(data=bool((cur_byte >> i) & 1)))

    def loop(self):
        try:
            # --- read inputs ---
            di = self.client.read_inputs_8()
            di_byte = bits_to_byte(di)

            # --- edge detect ---
            rising = (di_byte & (~self.prev_inputs_byte & 0xFF)) & 0xFF
            falling = (self.prev_inputs_byte & (~di_byte & 0xFF)) & 0xFF

            # per-bit DI (publish hanya saat berubah)
            self._publish_di_bits_changed(self.prev_inputs_byte, di_byte)

            # debounce sederhana + publish byte
            now = time.time()
            if self.last_inputs_byte is None or di_byte != self.last_inputs_byte:
                if (now - self.last_change_time) * 1000.0 >= self.debounce_ms:
                    self.last_inputs_byte = di_byte
                    self.last_change_time = now
                    self.pub_inputs.publish(UInt8(data=di_byte))
            else:
                self.pub_inputs.publish(UInt8(data=di_byte))

            # publish edge bitfield hanya saat non-zero + nama kanal
            if rising:
                self.pub_di_rising.publish(UInt8(data=rising))
                self.pub_di_rising_names.publish(String(data=names_from_mask(rising, self.di_names)))
            if falling:
                self.pub_di_falling.publish(UInt8(data=falling))
                self.pub_di_falling_names.publish(String(data=names_from_mask(falling, self.di_names)))

            # update prev DI
            self.prev_inputs_byte = di_byte

            # --- outputs (opsional) ---
            if self.publish_outputs and self.pub_outputs is not None:
                do = self.client.read_outputs_8()
                do_byte = bits_to_byte(do)
                # publish byte tiap siklus
                self.pub_outputs.publish(UInt8(data=do_byte))
                # per-bit DO (publish hanya saat berubah)
                self._publish_do_bits_changed(self.prev_outputs_byte, do_byte)
                self.prev_outputs_byte = do_byte

        except Exception as e:
            self.get_logger().warn(f"Polling error: {e}")

def main():
    rclpy.init()
    node = WaveshareIONode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
