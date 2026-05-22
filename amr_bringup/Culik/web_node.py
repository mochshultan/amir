# ===================== FASTAPI SETUP =====================
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from pydantic import BaseModel
from pathlib import Path
import subprocess
import time
import uvicorn
import rclpy, asyncio, json, threading
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt8, String
from std_srvs.srv import Empty
import math
from nav_msgs.msg import Odometry

# ===================== ROS2 NODE =====================
# ===================== ROS2 NODE =====================
class AryaWebNode(Node):
    def __init__(self):
        super().__init__('arya_web_node')

        self.declare_parameter("imu_serial_port", "/dev/ttyUSB0")
        self.declare_parameter("imu_serial_baud", 921600)
        self.declare_parameter("imu_respawn_wait_sec", 3.0)
        self.declare_parameter("imu_restart_fallback", True)
        self.declare_parameter("restart_ekf_on_reset_odom", True)
        self.declare_parameter("ekf_config", "~/arya_ws/src/sensor_tf_fusion/config/ekf.yaml")
        self.declare_parameter("ekf_respawn_wait_sec", 3.0)
        self.declare_parameter("ekf_restart_fallback", True)

        self.imu_serial_port = self.get_parameter("imu_serial_port").value
        self.imu_serial_baud = int(self.get_parameter("imu_serial_baud").value)
        self.imu_respawn_wait_sec = float(self.get_parameter("imu_respawn_wait_sec").value)
        self.imu_restart_fallback = bool(self.get_parameter("imu_restart_fallback").value)
        self.restart_ekf_on_reset_odom = bool(self.get_parameter("restart_ekf_on_reset_odom").value)
        self.ekf_config = str(Path(self.get_parameter("ekf_config").value).expanduser())
        self.ekf_respawn_wait_sec = float(self.get_parameter("ekf_respawn_wait_sec").value)
        self.ekf_restart_fallback = bool(self.get_parameter("ekf_restart_fallback").value)
        self.imu_restart_lock = threading.Lock()
        self.localization_reset_lock = threading.Lock()

        self.telemetry = [0, 0, 0, 0, 0, 0]
        self.odom = [0, 0, 0]

        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # ================= SUBSCRIBERS =================
        self.sub_telemetry = self.create_subscription(
            Float32MultiArray, 'telemetry', self.cb_telemetry, 10
        )

        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.cb_odom, 10
        )

        self.sub_io_inputs = self.create_subscription(
            UInt8, 'io/inputs_raw', self.cb_io_inputs, 10
        )

        self.sub_io_outputs = self.create_subscription(
            UInt8, 'io/outputs_raw', self.cb_io_outputs, 10
        )

        # ================= PUBLISHERS =================
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_manual', qos_reliable)
        self.pub_reset_odom = self.create_publisher(Float32MultiArray, 'reset_odom', qos_reliable)
        self.pub_reset_encoder = self.create_publisher(Float32MultiArray, 'reset_encoder', qos_reliable)
        self.pub_io_cmd = self.create_publisher(String, 'io/cmd_single', 10)
        self.pub_io_mask = self.create_publisher(UInt8, 'io/cmd_mask', 10)

        # ================= SERVICE CLIENTS =================
        self.lidar_start_client = self.create_client(Empty, 'start_motor')
        self.lidar_stop_client = self.create_client(Empty, 'stop_motor')

        # ================= DATA =================
        self.io_inputs_byte = 0
        self.io_outputs_byte = 0
        self.lidar_motor_enabled = True

        self.get_logger().info("✅ ARYA WebNode aktif & terhubung ke ROS2")

    # ================= CALLBACK =================
    def cb_telemetry(self, msg):
        self.telemetry = list(msg.data)

    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        theta = math.atan2(math.sin(theta), math.cos(theta))  # normalize
        self.odom = [x, y, theta]

    def cb_io_inputs(self, msg):
        self.io_inputs_byte = int(msg.data)

    def cb_io_outputs(self, msg):
        self.io_outputs_byte = int(msg.data)

    # ================= FIX UTAMA =================
    def send_cmd_vel(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)

        # Publish manual commands through twist_mux, not directly to final /cmd_vel.
        self.pub_cmd.publish(msg)

        # flush stop biar responsif
        if v == 0.0 and w == 0.0:
            self.pub_cmd.publish(msg)

    def reset_odom_and_restart_imu(self):
        self.pub_reset_odom.publish(Float32MultiArray(data=[0.0, 0.0, 0.0]))
        threading.Thread(target=self.reset_localization_nodes, daemon=True).start()

    def reset_localization_nodes(self):
        if not self.localization_reset_lock.acquire(blocking=False):
            self.get_logger().warn("Reset localization masih berjalan, request baru diabaikan")
            return

        try:
            self.restart_imu_node()
            if self.restart_ekf_on_reset_odom:
                self.restart_ekf_node()
        finally:
            self.localization_reset_lock.release()

    def restart_imu_node(self):
        if not self.imu_restart_lock.acquire(blocking=False):
            self.get_logger().warn("Restart IMU masih berjalan, request baru diabaikan")
            return

        try:
            old_pids = self.get_process_pids("/wheeltec_n100_imu/imu_node")
            killed = self.stop_process("/wheeltec_n100_imu/imu_node", "IMU")
            if killed:
                self.get_logger().info("Reset odom dikirim, menunggu imu_node respawn")
                self.wait_for_old_pids_to_exit("/wheeltec_n100_imu/imu_node", old_pids, 2.0)
            else:
                self.get_logger().warn("Reset odom dikirim, tapi proses imu_node tidak ditemukan")

            deadline = time.monotonic() + max(0.0, self.imu_respawn_wait_sec)
            while time.monotonic() < deadline:
                time.sleep(0.25)
                pids = self.get_process_pids("/wheeltec_n100_imu/imu_node")
                if (pids - old_pids) or (not old_pids and pids):
                    self.get_logger().info("imu_node sudah aktif kembali")
                    return

            if not self.imu_restart_fallback:
                self.get_logger().warn("imu_node belum respawn dan fallback start dimatikan")
                return

            self.start_imu_process()
        finally:
            self.imu_restart_lock.release()

    def restart_ekf_node(self):
        old_pids = self.get_process_pids("/robot_localization/ekf_node")
        killed = self.stop_process("/robot_localization/ekf_node", "EKF")
        if killed:
            self.get_logger().info("Menunggu ekf_node respawn agar yaw kembali 0")
            self.wait_for_old_pids_to_exit("/robot_localization/ekf_node", old_pids, 2.0)
        else:
            self.get_logger().warn("Proses ekf_node tidak ditemukan saat reset odom")

        deadline = time.monotonic() + max(0.0, self.ekf_respawn_wait_sec)
        while time.monotonic() < deadline:
            time.sleep(0.25)
            pids = self.get_process_pids("/robot_localization/ekf_node")
            if (pids - old_pids) or (not old_pids and pids):
                self.get_logger().info("ekf_node sudah aktif kembali")
                return

        if not self.ekf_restart_fallback:
            self.get_logger().warn("ekf_node belum respawn dan fallback start dimatikan")
            return

        self.start_ekf_process()

    def stop_process(self, pattern, label):
        try:
            result = subprocess.run(
                ["pkill", "-TERM", "-f", pattern],
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except FileNotFoundError:
            self.get_logger().error(f"Gagal stop {label}: command 'pkill' tidak ditemukan")
            return False
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"Gagal stop {label}: timeout saat menghentikan proses")
            return False

        if result.returncode in (0, 1):
            return result.returncode == 0

        stderr = result.stderr.strip()
        self.get_logger().error(f"Gagal menghentikan {label}: {stderr}")
        return False

    def get_process_pids(self, pattern):
        try:
            result = subprocess.run(
                ["pgrep", "-f", pattern],
                capture_output=True,
                text=True,
                timeout=1.0,
            )
            if result.returncode != 0:
                return set()
            return {int(pid) for pid in result.stdout.split() if pid.strip().isdigit()}
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return set()

    def wait_for_old_pids_to_exit(self, pattern, old_pids, timeout_sec):
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while old_pids and time.monotonic() < deadline:
            if not (self.get_process_pids(pattern) & old_pids):
                return True
            time.sleep(0.1)
        return not old_pids

    def start_imu_process(self):
        try:
            imu_executable = Path(get_package_prefix("wheeltec_n100_imu")) / "lib" / "wheeltec_n100_imu" / "imu_node"
        except Exception as exc:
            self.get_logger().error(f"Gagal mencari package wheeltec_n100_imu: {exc}")
            return

        if not imu_executable.exists():
            self.get_logger().error(f"Executable imu_node tidak ditemukan: {imu_executable}")
            return

        cmd = [
            str(imu_executable),
            "--ros-args",
            "-p", f"serial_port:={self.imu_serial_port}",
            "-p", f"serial_baud:={self.imu_serial_baud}",
        ]

        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception as exc:
            self.get_logger().error(f"Gagal start ulang imu_node: {exc}")
            return

        self.get_logger().info(f"imu_node distart ulang manual di {self.imu_serial_port}")

    def start_ekf_process(self):
        try:
            ekf_executable = Path(get_package_prefix("robot_localization")) / "lib" / "robot_localization" / "ekf_node"
        except Exception as exc:
            self.get_logger().error(f"Gagal mencari package robot_localization: {exc}")
            return

        if not ekf_executable.exists():
            self.get_logger().error(f"Executable ekf_node tidak ditemukan: {ekf_executable}")
            return

        if not Path(self.ekf_config).exists():
            self.get_logger().error(f"Config EKF tidak ditemukan: {self.ekf_config}")
            return

        cmd = [
            str(ekf_executable),
            "--ros-args",
            "-r", "__node:=ekf_filter_node",
            "--params-file", self.ekf_config,
        ]

        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception as exc:
            self.get_logger().error(f"Gagal start ulang ekf_node: {exc}")
            return

        self.get_logger().info("ekf_node distart ulang manual")

    def set_lidar_motor(self, enabled):
        client = self.lidar_start_client if enabled else self.lidar_stop_client
        service_name = 'start_motor' if enabled else 'stop_motor'

        if not client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(f"Service LIDAR '{service_name}' belum tersedia")
            return

        future = client.call_async(Empty.Request())
        future.add_done_callback(
            lambda done_future: self._on_lidar_motor_response(done_future, enabled, service_name)
        )

    def _on_lidar_motor_response(self, future, enabled, service_name):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f"Gagal memanggil service LIDAR '{service_name}': {exc}")
            return

        self.lidar_motor_enabled = bool(enabled)
        state = "ON" if enabled else "OFF"
        self.get_logger().info(f"Motor LIDAR {state}")
# ===================== FASTAPI APP =====================
app = FastAPI()
ROS_NODE: AryaWebNode = None
MAP_SELECTION_FILE = Path.home() / ".arya_amr" / "selected_localization_map.txt"
MAP_SELECTION_LOCK = threading.Lock()

# --- CARI STATIC FOLDER SECARA CERDAS ---
def find_static_folder() -> Path:
    """Cari folder 'static' di berbagai kemungkinan lokasi."""
    pkg_name = 'arya_web_interface'
    current_dir = Path(__file__).resolve().parent
    candidates = []

    # 1. lokasi build ROS2 (install/share)
    try:
        share_dir = Path(get_package_share_directory(pkg_name))
        candidates.append(share_dir / "static")
    except Exception:
        pass

    # 2. lokasi site-packages (ament_python)
    candidates.append(current_dir / "static")

    # 3. lokasi src (development mode)
    candidates.append(current_dir.parent / "static")
    candidates.append(current_dir.parents[1] / pkg_name / "static")

    # 4. manual fallback (hardcode jika pengembangan)
    candidates.append(Path.home() / "arya_ws/src/arya_web_interface/arya_web_interface/static")
    candidates.append(Path.home() / "awg_ws/src/arya_web_interface/arya_web_interface/static")

    for c in candidates:
        if c.exists() and (c / "index.html").exists():
            print(f"[INFO] ✅ STATIC folder ditemukan di: {c}")
            return c

    # Jika semua gagal, buat dummy
    fallback = current_dir / "static"
    fallback.mkdir(parents=True, exist_ok=True)
    (fallback / "index.html").write_text("<h1>ARYA Web Interface</h1>")
    print(f"[WARN] ⚠️ Tidak menemukan static folder asli, pakai fallback: {fallback}")
    return fallback


STATIC_DIR = find_static_folder()


def find_maps_folder() -> Path | None:
    """Cari folder map dari package amr_bringup."""
    current_dir = Path(__file__).resolve().parent
    candidates = []

    if len(current_dir.parents) > 1:
        candidates.append(current_dir.parents[1] / "amr_bringup" / "maps")

    candidates.append(Path.home() / "arya_ws" / "src" / "amr_bringup" / "maps")
    candidates.append(Path.home() / "awg_ws" / "src" / "amr_bringup" / "maps")

    try:
        bringup_share = Path(get_package_share_directory("amr_bringup"))
        candidates.append(bringup_share / "maps")
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists() and candidate.is_dir():
            return candidate

    return None


def find_localization_config() -> Path | None:
    """Cari file amcl.yaml untuk membaca map default saat startup."""
    current_dir = Path(__file__).resolve().parent
    candidates = []

    if len(current_dir.parents) > 1:
        candidates.append(current_dir.parents[1] / "amr_bringup" / "config" / "amcl.yaml")

    candidates.append(Path.home() / "arya_ws" / "src" / "amr_bringup" / "config" / "amcl.yaml")
    candidates.append(Path.home() / "awg_ws" / "src" / "amr_bringup" / "config" / "amcl.yaml")

    try:
        bringup_share = Path(get_package_share_directory("amr_bringup"))
        candidates.append(bringup_share / "config" / "amcl.yaml")
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists() and candidate.is_file():
            return candidate

    return None


def read_selected_map_path(maps_dir: Path | None = None) -> Path | None:
    try:
        raw_value = MAP_SELECTION_FILE.read_text(encoding="utf-8").strip()
    except OSError:
        return None

    if not raw_value:
        return None

    candidate = Path(raw_value).expanduser()
    if not candidate.is_absolute():
        if maps_dir is None:
            maps_dir = find_maps_folder()
        if maps_dir is None:
            return None
        candidate = maps_dir / candidate.name

    if candidate.exists() and candidate.is_file():
        return candidate

    return None


def extract_default_map_name(maps_dir: Path | None) -> str | None:
    if maps_dir is None:
        return None

    selected_map_path = read_selected_map_path(maps_dir)
    if selected_map_path is not None:
        return selected_map_path.name

    config_path = find_localization_config()
    if config_path:
        try:
            for line in config_path.read_text(encoding="utf-8").splitlines():
                if "yaml_filename:" not in line:
                    continue
                raw_value = line.split("yaml_filename:", 1)[1].strip().strip('"').strip("'")
                if not raw_value:
                    continue
                map_path = Path(raw_value).expanduser()
                if not map_path.is_absolute():
                    map_path = maps_dir / map_path.name
                elif not map_path.exists():
                    map_path = maps_dir / map_path.name
                if map_path.exists():
                    return map_path.name
        except OSError:
            pass

    available_maps = sorted(
        maps_dir.glob("*.yaml"),
        key=lambda path: path.name.casefold(),
    )
    return available_maps[0].name if available_maps else None


def list_available_maps() -> tuple[list[str], str | None]:
    maps_dir = find_maps_folder()
    if maps_dir is None:
        return [], None

    available_maps = sorted(
        [path.name for path in maps_dir.glob("*.yaml") if path.is_file()],
        key=str.casefold,
    )
    selected_map = extract_default_map_name(maps_dir)

    if selected_map not in available_maps:
        selected_map = available_maps[0] if available_maps else None

    return available_maps, selected_map


def resolve_map_path(map_name: str) -> Path:
    maps_dir = find_maps_folder()
    if maps_dir is None:
        raise FileNotFoundError("Folder map amr_bringup/maps tidak ditemukan")

    safe_name = Path(map_name).name
    if safe_name != map_name or not safe_name.endswith(".yaml"):
        raise ValueError("Nama map tidak valid")

    map_path = (maps_dir / safe_name).resolve()
    try:
        map_path.relative_to(maps_dir.resolve())
    except ValueError as exc:
        raise ValueError("Map berada di luar folder yang diizinkan") from exc

    if not map_path.exists():
        raise FileNotFoundError(f"Map '{safe_name}' tidak ditemukan")

    return map_path


def save_selected_map(map_path: Path):
    MAP_SELECTION_FILE.parent.mkdir(parents=True, exist_ok=True)
    try:
        MAP_SELECTION_FILE.write_text(str(map_path) + "\n", encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"Gagal menyimpan map pilihan: {exc}") from exc

    if ROS_NODE is not None:
        ROS_NODE.get_logger().info(f"Map localization dipilih: {map_path}")


class LocalizationRequest(BaseModel):
    map_name: str

# Mount folder static
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

# ===================== HTTP ROUTE =====================
@app.get("/")
async def index():
    return FileResponse(
        STATIC_DIR / "index.html",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
            "Expires": "0",
        },
    )


@app.get("/api/maps")
async def get_maps():
    maps, selected_map = list_available_maps()
    return {
        "maps": maps,
        "selected_map": selected_map,
    }


@app.post("/api/maps/select")
async def select_map(request: LocalizationRequest):
    try:
        map_path = resolve_map_path(request.map_name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc

    try:
        with MAP_SELECTION_LOCK:
            save_selected_map(map_path)
    except RuntimeError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc

    return {
        "message": f"Map {map_path.name} disimpan untuk localization manual.",
        "map_name": map_path.name,
    }


# ===================== WEBSOCKET HANDLER =====================
@app.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()
    last_cmd_time = 0.0
    last_v, last_w = 0.0, 0.0

    while True:
        try:
            # kirim telemetry cepat
            if ROS_NODE:
                await websocket.send_text(json.dumps({
                    "voltage": ROS_NODE.telemetry[0],
                    "current_left": ROS_NODE.telemetry[1],
                    "current_right": ROS_NODE.telemetry[2],
                    "temp_driver": ROS_NODE.telemetry[3],
                    "rpm_left": ROS_NODE.telemetry[4],
                    "rpm_right": ROS_NODE.telemetry[5],
                    "x": ROS_NODE.odom[0],
                    "y": ROS_NODE.odom[1],
                    "theta": ROS_NODE.odom[2],
                    "io_inputs":  ROS_NODE.io_inputs_byte,
                    "io_outputs": ROS_NODE.io_outputs_byte,
                    "lidar_motor": ROS_NODE.lidar_motor_enabled
                }))

            # baca perintah web
            msg = await asyncio.wait_for(websocket.receive_text(), timeout=0.05)
            data = json.loads(msg)

            if data.get("type") == "cmd_vel":
                v = float(data["linear"])
                w = float(data["angular"])
                last_cmd_time = asyncio.get_event_loop().time()
                last_v, last_w = v, w
                ROS_NODE.send_cmd_vel(v, w)

            elif data.get("type") == "reset_odom":
                ROS_NODE.reset_odom_and_restart_imu()
            elif data.get("type") == "reset_encoder":
                ROS_NODE.pub_reset_encoder.publish(Float32MultiArray(data=[0.0]))
            elif data.get("type") == "lidar_motor":
                ROS_NODE.set_lidar_motor(bool(data.get("enabled")))
            elif data.get("type") == "io_set_single":
                ch  = int(data["channel"])   # 1..8
                cmd = str(data["cmd"])       # "on" / "off"
                ROS_NODE.pub_io_cmd.publish(String(data=f"{cmd} {ch}"))

            elif data.get("type") == "io_mask":
                mask = int(data["mask"]) & 0xFF
                ROS_NODE.pub_io_mask.publish(UInt8(data=mask))

        except asyncio.TimeoutError:
            # tidak ada command baru dalam 0.1s → stop
            now = asyncio.get_event_loop().time()
            if now - last_cmd_time > 0.1 and (last_v != 0.0 or last_w != 0.0):
                ROS_NODE.send_cmd_vel(0.0, 0.0)
                last_v, last_w = 0.0, 0.0
            continue

        except WebSocketDisconnect:
            ROS_NODE.send_cmd_vel(0.0, 0.0)
            break


# ===================== ROS2 THREAD =====================
def ros_thread():
    global ROS_NODE
    rclpy.init()
    ROS_NODE = AryaWebNode()
    rclpy.spin(ROS_NODE)
    rclpy.shutdown()


# ===================== MAIN ENTRY =====================
def main():
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


if __name__ == "__main__":
    main()
