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
import os
import re
import signal
import shlex
import ast
import uuid
from action_msgs.msg import GoalStatus
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import UInt8, String
from std_srvs.srv import Empty
import math
import base64
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

try:
    from lifecycle_msgs.srv import GetState
except Exception:
    GetState = None

try:
    from rclpy.signals import SignalHandlerOptions
except Exception:
    SignalHandlerOptions = None

try:
    from rosidl_runtime_py.utilities import get_message
except Exception:
    get_message = None

MAX_ECHO_SEQUENCE_ITEMS = 16
MAX_ECHO_STRING_CHARS = 1200
MAX_ECHO_DEPTH = 3
MAX_LIDAR_POINTS = 720
NAV_GOAL_STATUS_LABELS = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}
LAUNCH_PRESETS = {
    "amir_hdl": {
        "label": "Hardware",
        "alias": "amir_hdl",
    },
    "local_hdl": {
        "label": "Localization",
        "alias": "local_hdl",
    },
    "nav_hdl": {
        "label": "Nav2",
        "alias": "nav_hdl",
    },
    "mapping": {
        "label": "Mapping",
        "alias": "mapping",
    },
}
KNOWN_ROS_TOPIC_TYPES = {
    "/cmd_vel": ["geometry_msgs/msg/Twist"],
    "/cmd_vel_manual": ["geometry_msgs/msg/Twist"],
    "/cmd_vel_auto": ["geometry_msgs/msg/Twist"],
    "/cmd_vel_raw": ["geometry_msgs/msg/Twist"],
    "/cmd_vel_nav": ["geometry_msgs/msg/Twist"],
    "/telemetry": ["std_msgs/msg/Float32MultiArray"],
    "/odom_data": ["std_msgs/msg/Float32MultiArray"],
    "/reset_odom": ["std_msgs/msg/Float32MultiArray"],
    "/reset_encoder": ["std_msgs/msg/Float32MultiArray"],
    "/io/inputs_raw": ["std_msgs/msg/UInt8"],
    "/io/outputs_raw": ["std_msgs/msg/UInt8"],
    "/io/di_rising": ["std_msgs/msg/UInt8"],
    "/io/di_falling": ["std_msgs/msg/UInt8"],
    "/io/di_rising_names": ["std_msgs/msg/String"],
    "/io/di_falling_names": ["std_msgs/msg/String"],
    "/io/cmd_single": ["std_msgs/msg/String"],
    "/io/cmd_mask": ["std_msgs/msg/UInt8"],
    "/map": ["nav_msgs/msg/OccupancyGrid"],
    "/scan": ["sensor_msgs/msg/LaserScan"],
    "/scan_fixed": ["sensor_msgs/msg/LaserScan"],
    "/odom": ["nav_msgs/msg/Odometry"],
    "/odometry/filtered": ["nav_msgs/msg/Odometry"],
    "/amcl_pose": ["geometry_msgs/msg/PoseWithCovarianceStamped"],
    "/goal_pose": ["geometry_msgs/msg/PoseStamped"],
    "/initialpose": ["geometry_msgs/msg/PoseWithCovarianceStamped"],
    "/plan": ["nav_msgs/msg/Path"],
    "/local_costmap/costmap": ["nav_msgs/msg/OccupancyGrid"],
    "/global_costmap/costmap": ["nav_msgs/msg/OccupancyGrid"],
    "/local_costmap/published_footprint": ["geometry_msgs/msg/PolygonStamped"],
    "/global_costmap/published_footprint": ["geometry_msgs/msg/PolygonStamped"],
    "/tf": ["tf2_msgs/msg/TFMessage"],
    "/tf_static": ["tf2_msgs/msg/TFMessage"],
}

for _channel in range(1, 9):
    KNOWN_ROS_TOPIC_TYPES[f"/io/di{_channel}"] = ["std_msgs/msg/Bool"]
    KNOWN_ROS_TOPIC_TYPES[f"/io/do{_channel}"] = ["std_msgs/msg/Bool"]


def normalize_topic_name(topic_name: str) -> str:
    clean_name = str(topic_name or "").strip()
    if not clean_name:
        return ""
    return clean_name if clean_name.startswith("/") else f"/{clean_name}"


def ros_value_to_bounded_data(value, depth: int = 0):
    if value is None or isinstance(value, (bool, int, float)):
        return value

    if isinstance(value, str):
        if len(value) <= MAX_ECHO_STRING_CHARS:
            return value
        return value[:MAX_ECHO_STRING_CHARS] + "...<truncated>"

    if isinstance(value, (bytes, bytearray)):
        sample = list(value[:MAX_ECHO_SEQUENCE_ITEMS])
        return {
            "__type": "bytes",
            "length": len(value),
            "sample": sample,
            "truncated": len(value) > len(sample),
        }

    if depth >= MAX_ECHO_DEPTH:
        return f"<{type(value).__name__}>"

    if hasattr(value, "get_fields_and_field_types"):
        result = {}
        for field_name in value.get_fields_and_field_types().keys():
            try:
                result[field_name] = ros_value_to_bounded_data(
                    getattr(value, field_name),
                    depth + 1,
                )
            except Exception as exc:
                result[field_name] = f"<unreadable: {exc}>"
        return result

    if isinstance(value, dict):
        result = {}
        for index, (key, item) in enumerate(value.items()):
            if index >= MAX_ECHO_SEQUENCE_ITEMS:
                result["..."] = f"{len(value) - index} more"
                break
            result[str(key)] = ros_value_to_bounded_data(item, depth + 1)
        return result

    if hasattr(value, "__len__") and hasattr(value, "__getitem__") and not isinstance(value, (str, bytes, bytearray)):
        try:
            length = len(value)
            limit = min(length, MAX_ECHO_SEQUENCE_ITEMS)
            sample = [
                ros_value_to_bounded_data(value[index], depth + 1)
                for index in range(limit)
            ]
            if length > limit:
                return {
                    "__type": type(value).__name__,
                    "length": length,
                    "sample": sample,
                    "truncated": True,
                }
            return sample
        except Exception:
            pass

    return str(value)

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
        self.declare_parameter("lidar_motor_enabled", True)
        self.declare_parameter("cmd_vel_topic", "cmd_vel_manual")
        self.declare_parameter("drive_mode_default", "auto")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "laser")

        self.imu_serial_port = self.get_parameter("imu_serial_port").value
        self.imu_serial_baud = int(self.get_parameter("imu_serial_baud").value)
        self.imu_respawn_wait_sec = float(self.get_parameter("imu_respawn_wait_sec").value)
        self.imu_restart_fallback = bool(self.get_parameter("imu_restart_fallback").value)
        self.restart_ekf_on_reset_odom = bool(self.get_parameter("restart_ekf_on_reset_odom").value)
        self.ekf_config = str(Path(self.get_parameter("ekf_config").value).expanduser())
        self.ekf_respawn_wait_sec = float(self.get_parameter("ekf_respawn_wait_sec").value)
        self.ekf_restart_fallback = bool(self.get_parameter("ekf_restart_fallback").value)
        self.lidar_motor_enabled = bool(self.get_parameter("lidar_motor_enabled").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value).strip() or "cmd_vel_manual"
        drive_mode_default = str(self.get_parameter("drive_mode_default").value).strip().lower()
        self.drive_mode = "manual" if drive_mode_default == "manual" else "auto"
        self.base_frame = str(self.get_parameter("base_frame").value).strip() or "base_link"
        self.laser_frame = str(self.get_parameter("laser_frame").value).strip() or "laser"
        self.imu_restart_lock = threading.Lock()
        self.localization_reset_lock = threading.Lock()
        self.topic_echo_lock = threading.Lock()
        self.drive_mode_lock = threading.Lock()
        self.launch_lock = threading.Lock()
        self.launch_processes = {}
        self.launch_log_dir = Path.home() / ".arya_amr" / "logs"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_lidar_tf_warn_time = 0.0
        self.last_grid_tf_warn_times = {}

        self.telemetry = [0, 0, 0, 0, 0, 0]
        self.odom = [0, 0, 0]
        self.amcl_pose = None
        self.nav_goal_lock = threading.Lock()
        self.mission_lock = threading.Lock()
        self.nav_goal_seq = 0
        self.mission_seq = 0
        self.current_nav_goal_handle = None
        self.nav_goal_contexts = {}
        self.nav_goal_status = {
            "state": "idle",
            "message": "Belum ada goal dari web.",
            "seq": 0,
        }
        self.station_mission_status = {
            "state": "idle",
            "message": "Belum ada station mission.",
            "mode": "idle",
            "mission_id": 0,
            "current_index": -1,
            "total": 0,
            "station": None,
        }

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

        self.sub_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl_pose, 10
        )

        self.sub_io_inputs = self.create_subscription(
            UInt8, 'io/inputs_raw', self.cb_io_inputs, 10
        )

        self.sub_io_outputs = self.create_subscription(
            UInt8, 'io/outputs_raw', self.cb_io_outputs, 10
        )

        # ================= PUBLISHERS =================
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, qos_reliable)
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

        self.map_data = None
        self.map_dirty = False
        self.local_costmap_data = None
        self.local_costmap_dirty = False
        self.global_costmap_data = None
        self.global_costmap_dirty = False
        self.path_data = None
        self.path_dirty = False
        self.lidar_scan_data = None
        self.lidar_scan_dirty = False

        qos_transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.cb_map, qos_transient)
        self.sub_local_costmap = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.cb_local_costmap, qos_best_effort)
        self.sub_global_costmap = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.cb_global_costmap, qos_transient)
        self.sub_path = self.create_subscription(NavPath, '/plan', self.cb_path, 1)
        self.sub_lidar_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_lidar_scan, qos_best_effort
        )

        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle_state_clients = {}
        if GetState is not None:
            for node_name in ("bt_navigator", "planner_server", "controller_server"):
                self.lifecycle_state_clients[node_name] = self.create_client(
                    GetState,
                    f"/{node_name}/get_state",
                )

        self.get_logger().info(
            f"ARYA WebNode aktif, cmd_vel topic: {self.cmd_vel_topic}, drive_mode: {self.drive_mode}"
        )

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

    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        theta = math.atan2(math.sin(theta), math.cos(theta))
        self.amcl_pose = [x, y, theta]

    def cb_io_inputs(self, msg):
        self.io_inputs_byte = int(msg.data)

    def cb_io_outputs(self, msg):
        self.io_outputs_byte = int(msg.data)

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        return math.atan2(math.sin(theta), math.cos(theta))

    def _parse_grid(self, msg):
        source_frame = (msg.header.frame_id or "").strip().lstrip("/") or "map"
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        oyaw = self.yaw_from_quaternion(msg.info.origin.orientation)
        transformed = self._transform_grid_origin_to_map(source_frame, ox, oy, oyaw)

        return {
            "frame_id": transformed["frame_id"],
            "source_frame_id": source_frame,
            "target_frame_id": "map",
            "transform_ok": transformed["transform_ok"],
            "w": msg.info.width,
            "h": msg.info.height,
            "res": msg.info.resolution,
            "ox": transformed["ox"],
            "oy": transformed["oy"],
            "oyaw": transformed["oyaw"],
            "b64": base64.b64encode(bytes(msg.data)).decode('ascii')
        }

    def _transform_grid_origin_to_map(self, source_frame, ox, oy, oyaw):
        if source_frame == "map":
            return {
                "frame_id": "map",
                "transform_ok": True,
                "ox": ox,
                "oy": oy,
                "oyaw": oyaw,
            }

        try:
            transform = self.tf_buffer.lookup_transform("map", source_frame, Time())
            tx = float(transform.transform.translation.x)
            ty = float(transform.transform.translation.y)
            tyaw = self.yaw_from_quaternion(transform.transform.rotation)
            cos_yaw = math.cos(tyaw)
            sin_yaw = math.sin(tyaw)
            return {
                "frame_id": "map",
                "transform_ok": True,
                "ox": tx + cos_yaw * ox - sin_yaw * oy,
                "oy": ty + sin_yaw * ox + cos_yaw * oy,
                "oyaw": math.atan2(math.sin(tyaw + oyaw), math.cos(tyaw + oyaw)),
            }
        except Exception as exc:
            now = time.monotonic()
            last_warn = self.last_grid_tf_warn_times.get(source_frame, 0.0)
            if now - last_warn > 5.0:
                self.get_logger().warn(
                    f"TF map <- {source_frame} belum tersedia untuk overlay grid: {exc}"
                )
                self.last_grid_tf_warn_times[source_frame] = now
            return {
                "frame_id": source_frame,
                "transform_ok": False,
                "ox": ox,
                "oy": oy,
                "oyaw": oyaw,
            }

    def cb_map(self, msg):
        self.map_data = self._parse_grid(msg)
        self.map_dirty = True

    def cb_local_costmap(self, msg):
        self.local_costmap_data = self._parse_grid(msg)
        self.local_costmap_dirty = True

    def cb_global_costmap(self, msg):
        self.global_costmap_data = self._parse_grid(msg)
        self.global_costmap_dirty = True

    def cb_path(self, msg):
        self.path_data = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        self.path_dirty = True

    def cb_lidar_scan(self, msg: LaserScan):
        ranges = list(msg.ranges)
        if not ranges:
            return

        source_frame = msg.header.frame_id.strip() if msg.header.frame_id else self.laser_frame
        target_frame = self.base_frame
        tx = ty = yaw = 0.0
        transformed_frame = source_frame
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            yaw = self.yaw_from_quaternion(transform.transform.rotation)
            transformed_frame = target_frame
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_lidar_tf_warn_time > 5.0:
                self.get_logger().warn(
                    f"TF {target_frame} <- {source_frame} belum tersedia untuk overlay scan: {exc}"
                )
                self.last_lidar_tf_warn_time = now

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        step = max(1, math.ceil(len(ranges) / MAX_LIDAR_POINTS))
        points = []
        range_min = float(msg.range_min or 0.0)
        range_max = float(msg.range_max or 0.0)
        for i in range(0, len(ranges), step):
            r = float(ranges[i])
            if not math.isfinite(r):
                continue
            if r <= range_min:
                continue
            if range_max > 0.0 and r >= range_max:
                continue
            angle = float(msg.angle_min + (i * msg.angle_increment))
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            bx = tx + cos_yaw * lx - sin_yaw * ly
            by = ty + sin_yaw * lx + cos_yaw * ly
            points.append([
                round(bx, 3),
                round(by, 3),
            ])

        self.lidar_scan_data = {
            "frame_id": transformed_frame,
            "source_frame_id": source_frame,
            "points": points,
        }
        self.lidar_scan_dirty = True

    def _make_goal_pose(self, x, y, theta):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(float(theta) / 2.0)
        msg.pose.orientation.w = math.cos(float(theta) / 2.0)
        return msg

    def _set_nav_goal_status(self, state, message, seq=None, **extra):
        with self.nav_goal_lock:
            active_seq = self.nav_goal_seq
            if seq is not None and seq != active_seq:
                return
            status = {
                "state": state,
                "message": message,
                "seq": active_seq if seq is None else seq,
            }
            status.update(extra)
            self.nav_goal_status = status

    def get_nav_goal_status(self):
        with self.nav_goal_lock:
            return dict(self.nav_goal_status)

    def _get_nav_goal_context(self, seq, remove=False):
        with self.nav_goal_lock:
            if remove:
                return self.nav_goal_contexts.pop(seq, {})
            return dict(self.nav_goal_contexts.get(seq, {}))

    def _set_station_mission_status(self, state, message, **extra):
        with self.mission_lock:
            status = {
                "state": state,
                "message": message,
                "mode": extra.pop("mode", self.station_mission_status.get("mode", "idle")),
                "mission_id": extra.pop("mission_id", self.station_mission_status.get("mission_id", 0)),
                "current_index": extra.pop("current_index", self.station_mission_status.get("current_index", -1)),
                "total": extra.pop("total", self.station_mission_status.get("total", 0)),
                "station": extra.pop("station", self.station_mission_status.get("station")),
            }
            status.update(extra)
            self.station_mission_status = status

    def get_station_mission_status(self):
        with self.mission_lock:
            return dict(self.station_mission_status)

    def _on_nav_goal_feedback(self, seq, feedback_msg):
        feedback = getattr(feedback_msg, "feedback", None)
        distance_remaining = getattr(feedback, "distance_remaining", None)
        extra = {}
        if distance_remaining is not None:
            extra["distance_remaining"] = round(float(distance_remaining), 3)
        context = self._get_nav_goal_context(seq)
        station = context.get("station")
        if context.get("source") == "queue":
            extra.update({
                "mission_mode": "queue",
                "mission_id": context.get("mission_id"),
                "mission_index": context.get("mission_index"),
                "station": station,
            })
            station_name = station.get("name", "station") if isinstance(station, dict) else "station"
            message = f"Menuju station {station_name}."
        else:
            message = "Goal sedang dieksekusi."
        self._set_nav_goal_status(seq=seq, state="executing", message=message, **extra)

    def _on_nav_goal_response(self, seq, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._set_nav_goal_status(
                seq=seq,
                state="failed",
                message=f"Gagal mengirim NavigateToPose goal: {exc}",
                mode="action",
            )
            self._handle_queue_goal_finished(seq, False, "failed", str(exc))
            return

        if not goal_handle.accepted:
            diagnostics = self.get_nav2_goal_diagnostics()
            hint = diagnostics.get("hint") or "Cek lifecycle bt_navigator dan log Nav2."
            self._set_nav_goal_status(
                seq=seq,
                state="rejected",
                message=f"NavigateToPose menolak goal dari web. {hint}",
                mode="action",
                diagnostics=diagnostics,
            )
            self._handle_queue_goal_finished(seq, False, "rejected", f"NavigateToPose menolak goal. {hint}")
            return

        with self.nav_goal_lock:
            if seq == self.nav_goal_seq:
                self.current_nav_goal_handle = goal_handle

        self._set_nav_goal_status(
            seq=seq,
            state="accepted",
            message="NavigateToPose menerima goal dari web.",
            mode="action",
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done_future: self._on_nav_goal_result(seq, done_future))

    def _on_nav_goal_result(self, seq, future):
        try:
            result = future.result()
            status_code = int(result.status)
        except Exception as exc:
            self._set_nav_goal_status(
                seq=seq,
                state="failed",
                message=f"Gagal membaca hasil NavigateToPose: {exc}",
                mode="action",
            )
            self._handle_queue_goal_finished(seq, False, "failed", str(exc))
            return

        state = NAV_GOAL_STATUS_LABELS.get(status_code, f"status_{status_code}")
        message = f"NavigateToPose {state}."
        context = self._get_nav_goal_context(seq, remove=True)
        extra = {}
        if context.get("source") == "queue":
            extra.update({
                "mission_mode": "queue",
                "mission_id": context.get("mission_id"),
                "mission_index": context.get("mission_index"),
                "station": context.get("station"),
            })
        self._set_nav_goal_status(
            seq=seq,
            state=state,
            message=message,
            mode="action",
            result_status=status_code,
            **extra,
        )
        with self.nav_goal_lock:
            if seq == self.nav_goal_seq:
                self.current_nav_goal_handle = None
        self._handle_queue_goal_finished(
            seq,
            status_code == GoalStatus.STATUS_SUCCEEDED,
            state,
            message,
            context=context,
        )

    def get_lifecycle_state(self, node_name: str, timeout_sec: float = 0.15) -> dict:
        client = self.lifecycle_state_clients.get(node_name)
        if client is None or GetState is None:
            return {
                "node": node_name,
                "available": False,
                "state": "unknown",
                "message": "Lifecycle service client tidak tersedia.",
            }

        if not client.wait_for_service(timeout_sec=0.02):
            return {
                "node": node_name,
                "available": False,
                "state": "unknown",
                "message": f"Service /{node_name}/get_state belum tersedia.",
            }

        future = client.call_async(GetState.Request())
        deadline = time.monotonic() + max(0.02, timeout_sec)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            return {
                "node": node_name,
                "available": True,
                "state": "unknown",
                "message": f"Timeout membaca lifecycle {node_name}.",
            }

        try:
            response = future.result()
            state = response.current_state
            label = str(getattr(state, "label", "") or "unknown")
            return {
                "node": node_name,
                "available": True,
                "state": label,
                "state_id": int(getattr(state, "id", 0)),
                "message": f"{node_name} lifecycle={label}.",
            }
        except Exception as exc:
            return {
                "node": node_name,
                "available": True,
                "state": "unknown",
                "message": f"Gagal membaca lifecycle {node_name}: {exc}",
            }

    def get_nav2_goal_diagnostics(self) -> dict:
        states = {
            node_name: self.get_lifecycle_state(node_name, timeout_sec=0.08)
            for node_name in ("bt_navigator", "planner_server", "controller_server")
        }
        bt_state = states.get("bt_navigator", {})
        bt_label = str(bt_state.get("state") or "unknown")

        if bt_state.get("available") and bt_label != "active":
            hint = f"bt_navigator belum active (state: {bt_label}). Tunggu Nav2 lifecycle aktif atau restart Nav2."
        elif self.amcl_pose is None:
            hint = "AMCL pose belum masuk; set initial pose dan pastikan localization aktif."
        else:
            hint = "bt_navigator aktif; cek log bt_navigator untuk BT XML atau goal yang tidak valid."

        return {
            "lifecycle": states,
            "amcl_pose_available": self.amcl_pose is not None,
            "map_available": self.map_data is not None,
            "hint": hint,
        }

    def wait_for_nav2_goal_ready(self, timeout_sec: float = 2.0) -> dict:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        last_diagnostics = {}

        while time.monotonic() <= deadline:
            action_ready = self.nav_to_pose_client.wait_for_server(timeout_sec=0.05)
            diagnostics = self.get_nav2_goal_diagnostics()
            last_diagnostics = diagnostics

            bt_state = diagnostics.get("lifecycle", {}).get("bt_navigator", {})
            bt_available = bool(bt_state.get("available"))
            bt_label = str(bt_state.get("state") or "unknown")
            lifecycle_ok = (not bt_available) or bt_label == "active"

            if action_ready and lifecycle_ok:
                return {
                    "ok": True,
                    "action_ready": True,
                    "diagnostics": diagnostics,
                    "message": "NavigateToPose ready.",
                }

            time.sleep(0.1)

        hint = last_diagnostics.get("hint") or "Action navigate_to_pose belum ready."
        return {
            "ok": False,
            "action_ready": False,
            "diagnostics": last_diagnostics,
            "message": hint,
        }

    def send_goal(
        self,
        x,
        y,
        theta,
        *,
        source="single",
        mission_id=None,
        mission_index=None,
        station=None,
        clear_mission=True,
        allow_topic_fallback=True,
    ):
        msg = self._make_goal_pose(x, y, theta)
        self.set_drive_mode("auto")
        self.path_data = []
        self.path_dirty = True

        if clear_mission:
            self._set_station_mission_status(
                "idle",
                "Single mission aktif.",
                mode="single",
                mission_id=0,
                current_index=-1,
                total=0,
                station=station,
            )

        with self.nav_goal_lock:
            self.nav_goal_seq += 1
            seq = self.nav_goal_seq
            context = {
                "source": source,
                "mission_id": mission_id,
                "mission_index": mission_index,
                "station": station,
            }
            self.nav_goal_contexts[seq] = context
            self.nav_goal_status = {
                "state": "queued",
                "message": "Goal web disiapkan.",
                "seq": seq,
                "mode": "action",
            }
            if station:
                self.nav_goal_status["station"] = station
            if source == "queue":
                self.nav_goal_status["mission_mode"] = "queue"
                self.nav_goal_status["mission_id"] = mission_id
                self.nav_goal_status["mission_index"] = mission_index

        readiness = self.wait_for_nav2_goal_ready(timeout_sec=2.0)
        if not readiness.get("ok"):
            if not allow_topic_fallback:
                self._set_nav_goal_status(
                    seq=seq,
                    state="failed",
                    message=readiness.get("message", "Action navigate_to_pose belum ready."),
                    mode="action",
                    diagnostics=readiness.get("diagnostics", {}),
                )
                self._handle_queue_goal_finished(seq, False, "failed", readiness.get("message", "Action navigate_to_pose belum ready."))
                return {
                    "ok": False,
                    "mode": "action",
                    "seq": seq,
                    "message": readiness.get("message", "Action navigate_to_pose belum ready."),
                    "diagnostics": readiness.get("diagnostics", {}),
                }

            diagnostics = readiness.get("diagnostics", {})
            bt_state = diagnostics.get("lifecycle", {}).get("bt_navigator", {})
            if bt_state.get("available") and str(bt_state.get("state") or "unknown") != "active":
                message = readiness.get("message", "bt_navigator belum active.")
                self._set_nav_goal_status(
                    seq=seq,
                    state="failed",
                    message=message,
                    mode="action",
                    diagnostics=diagnostics,
                )
                return {
                    "ok": False,
                    "mode": "action",
                    "seq": seq,
                    "message": message,
                    "diagnostics": diagnostics,
                }

            self.pub_goal.publish(msg)
            self._set_nav_goal_status(
                seq=seq,
                state="fallback_published",
                message="Action navigate_to_pose belum ready; goal dipublish ke /goal_pose.",
                mode="topic",
                diagnostics=readiness.get("diagnostics", {}),
            )
            self.get_logger().warn("NavigateToPose action belum ready, fallback publish /goal_pose")
            return {
                "ok": True,
                "mode": "topic",
                "seq": seq,
                "message": "Goal dikirim via fallback /goal_pose.",
                "diagnostics": readiness.get("diagnostics", {}),
            }

        goal = NavigateToPose.Goal()
        goal.pose = msg
        send_future = self.nav_to_pose_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback_msg: self._on_nav_goal_feedback(seq, feedback_msg),
        )
        send_future.add_done_callback(lambda future: self._on_nav_goal_response(seq, future))
        self._set_nav_goal_status(
            seq=seq,
            state="sent",
            message="Goal dikirim ke NavigateToPose action.",
            mode="action",
            mission_mode="queue" if source == "queue" else None,
            mission_id=mission_id,
            mission_index=mission_index,
            station=station,
        )
        self.get_logger().info(
            f"Goal pose dari web: x={float(x):.3f}, y={float(y):.3f}, theta={math.degrees(float(theta)):.1f} deg"
        )
        return {
            "ok": True,
            "mode": "action",
            "seq": seq,
            "message": "Goal dikirim ke NavigateToPose.",
        }

    def _normalize_mission_station(self, item, index=0):
        if not isinstance(item, dict):
            raise ValueError("Station mission harus berupa object.")

        station_id = str(item.get("id") or f"station_{index + 1}").strip()[:80]
        name = str(item.get("name") or f"Station {index + 1}").strip()[:80]
        try:
            x = float(item.get("x"))
            y = float(item.get("y"))
            theta = float(item.get("theta"))
            wait_sec = float(item.get("wait_sec", 0.0) or 0.0)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Station {name} punya nilai koordinat tidak valid.") from exc
        enabled = bool(item.get("enabled", True))

        if not all(math.isfinite(value) for value in (x, y, theta, wait_sec)):
            raise ValueError(f"Station {name} punya nilai koordinat tidak valid.")
        if wait_sec < 0:
            raise ValueError(f"Station {name} punya wait_sec negatif.")
        if not enabled:
            raise ValueError(f"Station {name} tidak aktif.")

        return {
            "id": station_id,
            "name": name,
            "x": x,
            "y": y,
            "theta": theta,
            "wait_sec": wait_sec,
            "enabled": enabled,
        }

    def start_station_queue(self, stations):
        if not isinstance(stations, list):
            return {
                "ok": False,
                "message": "Payload stations harus list.",
            }

        try:
            queue = [
                self._normalize_mission_station(item, index)
                for index, item in enumerate(stations)
            ]
        except (TypeError, ValueError) as exc:
            return {
                "ok": False,
                "message": str(exc),
            }

        if not queue:
            return {
                "ok": False,
                "message": "Queue station masih kosong.",
            }

        self.cancel_current_nav_goal(update_status=False)
        with self.mission_lock:
            self.mission_seq += 1
            mission_id = self.mission_seq
            self.station_mission_status = {
                "state": "starting",
                "message": "Queue mission disiapkan.",
                "mode": "queue",
                "mission_id": mission_id,
                "queue": queue,
                "current_index": -1,
                "total": len(queue),
                "station": None,
                "stop_requested": False,
            }

        self._start_next_station_in_queue(mission_id)
        return {
            "ok": True,
            "message": f"Queue mission dimulai ({len(queue)} station).",
            "mission_status": self.get_station_mission_status(),
        }

    def cancel_current_nav_goal(self, update_status=True):
        with self.nav_goal_lock:
            goal_handle = self.current_nav_goal_handle
            seq = self.nav_goal_seq

        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Gagal cancel goal aktif: {exc}")

        if update_status:
            self._set_nav_goal_status(
                "canceling",
                "Cancel goal diminta dari web.",
                seq=seq,
                mode="action",
            )

    def cancel_station_queue(self):
        with self.mission_lock:
            status = self.station_mission_status
            if status.get("mode") != "queue" or status.get("state") not in ("starting", "running", "executing"):
                return {
                    "ok": True,
                    "message": "Tidak ada queue mission aktif.",
                    "mission_status": dict(status),
                }
            mission_id = status.get("mission_id", 0)
            self.station_mission_status = {
                **status,
                "state": "canceled",
                "message": "Queue mission dibatalkan dari web.",
                "stop_requested": True,
            }

        self.cancel_current_nav_goal(update_status=True)
        self.get_logger().info(f"Queue mission #{mission_id} dibatalkan dari web")
        return {
            "ok": True,
            "message": "Queue mission dibatalkan.",
            "mission_status": self.get_station_mission_status(),
        }

    def _start_next_station_in_queue(self, mission_id):
        with self.mission_lock:
            status = self.station_mission_status
            if status.get("mission_id") != mission_id or status.get("mode") != "queue":
                return
            if status.get("stop_requested"):
                return
            queue = list(status.get("queue") or [])
            next_index = int(status.get("current_index", -1)) + 1
            if next_index >= len(queue):
                self.station_mission_status = {
                    **status,
                    "state": "succeeded",
                    "message": "Queue mission selesai.",
                    "current_index": len(queue) - 1,
                    "total": len(queue),
                    "station": None,
                }
                self._set_nav_goal_status(
                    "succeeded",
                    "Queue mission selesai.",
                    mode="queue",
                    mission_id=mission_id,
                    mission_index=len(queue) - 1,
                )
                return

            station = queue[next_index]
            self.station_mission_status = {
                **status,
                "state": "executing",
                "message": f"Menuju station {station['name']}.",
                "current_index": next_index,
                "total": len(queue),
                "station": station,
            }

        result = self.send_goal(
            station["x"],
            station["y"],
            station["theta"],
            source="queue",
            mission_id=mission_id,
            mission_index=next_index,
            station=station,
            clear_mission=False,
            allow_topic_fallback=False,
        )
        if not result.get("ok"):
            self._abort_station_queue(mission_id, next_index, station, result.get("message", "Goal gagal dikirim."))

    def _abort_station_queue(self, mission_id, index, station, reason):
        with self.mission_lock:
            status = self.station_mission_status
            if status.get("mission_id") != mission_id:
                return
            self.station_mission_status = {
                **status,
                "state": "failed",
                "message": reason,
                "current_index": index,
                "station": station,
                "failed_index": index,
                "failed_station": station,
            }
        self.get_logger().warn(f"Queue mission #{mission_id} gagal di index {index}: {reason}")

    def _handle_queue_goal_finished(self, seq, success, state, message, context=None):
        if context is None:
            context = self._get_nav_goal_context(seq, remove=True)
        if context.get("source") != "queue":
            return

        mission_id = context.get("mission_id")
        index = context.get("mission_index")
        station = context.get("station")
        with self.mission_lock:
            status = self.station_mission_status
            if status.get("mission_id") != mission_id or status.get("mode") != "queue":
                return
            if status.get("stop_requested"):
                return

        if success:
            self._start_next_station_in_queue(mission_id)
        else:
            station_name = station.get("name", "station") if isinstance(station, dict) else "station"
            self._abort_station_queue(
                mission_id,
                int(index if index is not None else -1),
                station,
                f"Queue abort di {station_name}: {message or state}.",
            )

    def set_initial_pose(self, x, y, theta):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(theta) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(theta) / 2.0)

        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.06853892326654787
        msg.pose.covariance = covariance

        self.pub_initial_pose.publish(msg)
        self.amcl_pose = [float(x), float(y), float(theta)]
        self.get_logger().info(
            f"Initial pose dari web: x={float(x):.3f}, y={float(y):.3f}, theta={math.degrees(float(theta)):.1f} deg"
        )
        return {
            "ok": True,
            "message": "Initial pose dipublish ke /initialpose.",
            "x": float(x),
            "y": float(y),
            "theta": float(theta),
        }

    def list_ros_topics(self):
        topic_map = {}
        try:
            topics = self.get_topic_names_and_types()
        except Exception as exc:
            self.get_logger().warn(f"Gagal membaca ROS topic list: {exc}")
            topics = []

        for name, types in topics:
            clean_name = normalize_topic_name(name)
            if not clean_name:
                continue
            topic_map[clean_name] = {
                "name": clean_name,
                "types": sorted(set(types)),
                "available": True,
                "source": "graph",
            }

        for name, types in self.known_ros_topic_types().items():
            if name in topic_map:
                merged_types = set(topic_map[name]["types"])
                merged_types.update(types)
                topic_map[name]["types"] = sorted(merged_types)
                topic_map[name]["known"] = True
            else:
                topic_map[name] = {
                    "name": name,
                    "types": sorted(set(types)),
                    "available": False,
                    "source": "known",
                    "known": True,
                }

        items = list(topic_map.values())
        return sorted(items, key=lambda item: item["name"].casefold())

    def known_ros_topic_types(self):
        topics = {
            normalize_topic_name(name): list(types)
            for name, types in KNOWN_ROS_TOPIC_TYPES.items()
        }
        configured_cmd_vel = normalize_topic_name(self.cmd_vel_topic)
        if configured_cmd_vel:
            topics.setdefault(configured_cmd_vel, [])
            if "geometry_msgs/msg/Twist" not in topics[configured_cmd_vel]:
                topics[configured_cmd_vel].append("geometry_msgs/msg/Twist")
        return topics

    def resolve_topic_types(self, topic_name: str) -> list[str]:
        clean_name = normalize_topic_name(topic_name)
        if not clean_name:
            return []

        try:
            for name, types in self.get_topic_names_and_types():
                if normalize_topic_name(name) == clean_name:
                    return sorted(set(types))
        except Exception as exc:
            self.get_logger().warn(f"Gagal resolve topic '{clean_name}' dari ROS graph: {exc}")

        return sorted(set(self.known_ros_topic_types().get(clean_name, [])))

    def create_echo_subscription(self, topic_name: str, callback):
        if get_message is None:
            raise RuntimeError("rosidl_runtime_py tidak tersedia untuk dynamic topic echo")

        clean_name = normalize_topic_name(topic_name)
        if not clean_name:
            raise ValueError("Nama topic tidak valid")

        msg_types = self.resolve_topic_types(clean_name)
        if not msg_types:
            raise ValueError(
                f"Topic '{clean_name}' tidak ditemukan di ROS graph dan belum ada tipe fallback."
            )

        msg_type = msg_types[0]
        msg_cls = get_message(msg_type)
        qos_echo = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        with self.topic_echo_lock:
            subscription = self.create_subscription(msg_cls, clean_name, callback, qos_echo)

        return subscription, msg_type

    def destroy_echo_subscription(self, subscription):
        if subscription is None:
            return
        with self.topic_echo_lock:
            self.destroy_subscription(subscription)

    def publish_cmd_vel(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)

        self.pub_cmd.publish(msg)

        # flush stop biar responsif
        if v == 0.0 and w == 0.0:
            self.pub_cmd.publish(msg)

    # ================= FIX UTAMA =================
    def send_cmd_vel(self, v, w):
        with self.drive_mode_lock:
            if self.drive_mode != "manual":
                return False

        self.publish_cmd_vel(v, w)
        return True

    def set_drive_mode(self, mode):
        next_mode = str(mode or "").strip().lower()
        if next_mode not in ("auto", "manual"):
            self.get_logger().warn(f"Mode drive tidak valid: {mode}")
            return self.drive_mode

        with self.drive_mode_lock:
            previous_mode = self.drive_mode
            if previous_mode == next_mode:
                return self.drive_mode

            if previous_mode == "manual" and next_mode == "auto":
                self.publish_cmd_vel(0.0, 0.0)

            self.drive_mode = next_mode

        self.get_logger().info(f"Drive mode: {previous_mode} -> {next_mode}")
        return next_mode

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

    def is_localization_ready(self):
        if self.amcl_pose is not None:
            return True

        try:
            topic_names = {name for name, _types in self.get_topic_names_and_types()}
        except Exception:
            return False

        return "/map" in topic_names and "/amcl_pose" in topic_names

    def is_hardware_ready(self):
        try:
            topic_names = {name for name, _types in self.get_topic_names_and_types()}
        except Exception:
            return False

        return "/scan" in topic_names and "/odometry/filtered" in topic_names

    def _default_launch_status(self, name):
        preset = LAUNCH_PRESETS[name]
        return {
            "name": name,
            "label": preset["label"],
            "status": "stopped",
            "running": False,
            "pid": None,
            "returncode": None,
            "log_path": "",
            "message": "Stopped",
        }

    def _launch_status_locked(self, name):
        status = self._default_launch_status(name)
        state = self.launch_processes.get(name)
        if not state:
            return status

        process = state.get("process")
        returncode = process.poll() if process else state.get("returncode")
        running = returncode is None
        stop_requested = bool(state.get("stop_requested"))

        status.update({
            "running": running,
            "pid": process.pid if process else state.get("pid"),
            "returncode": returncode,
            "log_path": str(state.get("log_path") or ""),
        })

        if running:
            status["status"] = state.get("status", "running")
            status["message"] = state.get("message", "Running")
        else:
            state["returncode"] = returncode
            if returncode == 0 or stop_requested:
                status["status"] = "stopped"
                status["message"] = "Stopped"
            else:
                status["status"] = "failed"
                status["message"] = f"Exited with code {returncode}"
            state["status"] = status["status"]
            state["message"] = status["message"]

        return status

    def get_launch_statuses(self):
        with self.launch_lock:
            return [
                self._launch_status_locked(name)
                for name in LAUNCH_PRESETS.keys()
            ]

    def _is_launch_running_locked(self, name):
        state = self.launch_processes.get(name)
        process = state.get("process") if state else None
        return bool(process and process.poll() is None)

    def start_launch_preset(self, name):
        clean_name = str(name or "").strip()
        if clean_name not in LAUNCH_PRESETS:
            return {
                "ok": False,
                "name": clean_name,
                "action": "start",
                "message": "Launch preset tidak dikenal.",
            }

        if clean_name == "local_hdl":
            with self.launch_lock:
                hardware_running = self._is_launch_running_locked("amir_hdl")
            if not hardware_running and not self.is_hardware_ready():
                return {
                    "ok": False,
                    "name": clean_name,
                    "action": "start",
                    "message": "Hardware belum terdeteksi. Jalankan amir_hdl dulu.",
                }

        if clean_name == "nav_hdl":
            with self.launch_lock:
                local_running = self._is_launch_running_locked("local_hdl")
            if not local_running and not self.is_localization_ready():
                return {
                    "ok": False,
                    "name": clean_name,
                    "action": "start",
                    "message": "Localization belum terdeteksi. Jalankan local_hdl dulu.",
                }

        preset = LAUNCH_PRESETS[clean_name]
        with self.launch_lock:
            current = self._launch_status_locked(clean_name)
            if current["running"]:
                return {
                    "ok": True,
                    "name": clean_name,
                    "action": "start",
                    "message": f"{preset['label']} sudah running.",
                    "status": current,
                }

            try:
                self.launch_log_dir.mkdir(parents=True, exist_ok=True)
            except Exception as exc:
                return {
                    "ok": False,
                    "name": clean_name,
                    "action": "start",
                    "message": f"Gagal membuat folder log: {exc}",
                }

            stamp = time.strftime("%Y%m%d_%H%M%S")
            log_path = self.launch_log_dir / f"{clean_name}_{stamp}.log"
            cmd = ["bash", "-ic", preset["alias"]]

            try:
                with log_path.open("ab") as log_file:
                    header = (
                        f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} "
                        f"START {clean_name}: {preset['alias']} ===\n"
                    )
                    log_file.write(header.encode("utf-8"))
                    process = subprocess.Popen(
                        cmd,
                        stdin=subprocess.DEVNULL,
                        stdout=log_file,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
            except Exception as exc:
                self.get_logger().error(f"Gagal start {clean_name}: {exc}")
                return {
                    "ok": False,
                    "name": clean_name,
                    "action": "start",
                    "message": f"Gagal start {preset['label']}: {exc}",
                }

            self.launch_processes[clean_name] = {
                "process": process,
                "pid": process.pid,
                "log_path": log_path,
                "status": "running",
                "message": "Running",
                "stop_requested": False,
            }
            time.sleep(0.2)
            status = self._launch_status_locked(clean_name)

        if status["status"] == "failed":
            message = f"{preset['label']} gagal start. Cek log: {log_path}"
            self.get_logger().error(message)
            return {
                "ok": False,
                "name": clean_name,
                "action": "start",
                "message": message,
                "status": status,
            }

        self.get_logger().info(f"{preset['label']} started via alias {preset['alias']} pid={process.pid}")
        return {
            "ok": True,
            "name": clean_name,
            "action": "start",
            "message": f"{preset['label']} started.",
            "status": status,
        }

    def _send_launch_signal(self, process, sig):
        try:
            if os.name == "nt":
                if sig == signal.SIGINT:
                    process.terminate()
                else:
                    process.kill()
            else:
                os.killpg(process.pid, sig)
            return True
        except ProcessLookupError:
            return True
        except Exception as exc:
            self.get_logger().warn(f"Gagal mengirim signal ke launch pid={process.pid}: {exc}")
            return False

    def stop_launch_preset(self, name):
        clean_name = str(name or "").strip()
        if clean_name not in LAUNCH_PRESETS:
            return {
                "ok": False,
                "name": clean_name,
                "action": "stop",
                "message": "Launch preset tidak dikenal.",
            }

        if clean_name == "amir_hdl":
            cascade_results = []
            for dependent_name in ("nav_hdl", "local_hdl", "mapping"):
                result = self.stop_launch_preset(dependent_name)
                cascade_results.append(result)
                if not result.get("ok"):
                    return {
                        "ok": False,
                        "name": clean_name,
                        "action": "stop",
                        "message": f"Gagal stop {dependent_name} sebelum stop Hardware.",
                        "cascade": cascade_results,
                    }

        preset = LAUNCH_PRESETS[clean_name]
        with self.launch_lock:
            state = self.launch_processes.get(clean_name)
            process = state.get("process") if state else None
            if not process or process.poll() is not None:
                status = self._launch_status_locked(clean_name)
                return {
                    "ok": True,
                    "name": clean_name,
                    "action": "stop",
                    "message": f"{preset['label']} sudah stopped.",
                    "status": status,
                }

            state["stop_requested"] = True
            state["status"] = "stopping"
            state["message"] = "Stopping"

        self._send_launch_signal(process, signal.SIGINT)
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            self._send_launch_signal(process, signal.SIGTERM)
            try:
                process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                self._send_launch_signal(process, signal.SIGKILL)
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    pass

        with self.launch_lock:
            status = self._launch_status_locked(clean_name)

        self.get_logger().info(f"{preset['label']} stop requested, status={status['status']}")
        return {
            "ok": status["status"] != "failed",
            "name": clean_name,
            "action": "stop",
            "message": f"{preset['label']} stopped." if status["status"] != "failed" else status["message"],
            "status": status,
        }
# ===================== FASTAPI APP =====================
app = FastAPI()
ROS_NODE: AryaWebNode = None
ROS_THREAD_STARTED = False
MAP_SELECTION_FILE = Path.home() / ".arya_amr" / "selected_localization_map.txt"
KEEPOUT_SELECTION_FILE = Path.home() / ".arya_amr" / "selected_keepout_mask.txt"
MAP_SELECTION_LOCK = threading.Lock()
MAPPING_SAVE_DIR = Path.home() / "arya_ws" / "src" / "amr_bringup_headless" / "maps"
MAPPING_SAVE_LOCK = threading.Lock()

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
MAIN_UI_FILE = "new.html" if (STATIC_DIR / "new.html").exists() else "index.html"


def find_maps_folder() -> Path | None:
    """Cari folder map dari package amr_bringup_headless."""
    current_dir = Path(__file__).resolve().parent
    candidates = []

    if len(current_dir.parents) > 1:
        candidates.append(current_dir.parents[1] / "amr_bringup_headless" / "maps")

    candidates.append(Path.home() / "arya_ws" / "src" / "amr_bringup_headless" / "maps")
    candidates.append(Path.home() / "awg_ws" / "src" / "amr_bringup_headless" / "maps")

    try:
        bringup_share = Path(get_package_share_directory("amr_bringup_headless"))
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
        candidates.append(current_dir.parents[1] / "amr_bringup_headless" / "config" / "amcl.yaml")

    candidates.append(Path.home() / "arya_ws" / "src" / "amr_bringup_headless" / "config" / "amcl.yaml")
    candidates.append(Path.home() / "awg_ws" / "src" / "amr_bringup_headless" / "config" / "amcl.yaml")

    try:
        bringup_share = Path(get_package_share_directory("amr_bringup_headless"))
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
        [
            path
            for path in maps_dir.glob("*.yaml")
            if path.is_file() and not path.name.endswith("_keepout.yaml")
        ],
        key=lambda path: path.name.casefold(),
    )
    return available_maps[0].name if available_maps else None


def list_available_maps() -> tuple[list[str], str | None]:
    maps_dir = find_maps_folder()
    if maps_dir is None:
        return [], None

    available_maps = sorted(
        [
            path.name
            for path in maps_dir.glob("*.yaml")
            if path.is_file() and not path.name.endswith("_keepout.yaml")
        ],
        key=str.casefold,
    )
    selected_map = extract_default_map_name(maps_dir)

    if selected_map not in available_maps:
        selected_map = available_maps[0] if available_maps else None

    return available_maps, selected_map


def resolve_map_path(map_name: str) -> Path:
    maps_dir = find_maps_folder()
    if maps_dir is None:
        raise FileNotFoundError("Folder map amr_bringup_headless/maps tidak ditemukan")

    safe_name = Path(map_name).name
    if safe_name != map_name or not safe_name.endswith(".yaml") or safe_name.endswith("_keepout.yaml"):
        raise ValueError("Nama map tidak valid")

    map_path = (maps_dir / safe_name).resolve()
    try:
        map_path.relative_to(maps_dir.resolve())
    except ValueError as exc:
        raise ValueError("Map berada di luar folder yang diizinkan") from exc

    if not map_path.exists():
        raise FileNotFoundError(f"Map '{safe_name}' tidak ditemukan")

    return map_path


def save_selected_keepout_for_map(map_path: Path):
    keepout_path = map_path.with_name(f"{map_path.stem}_keepout.yaml")
    if not keepout_path.exists():
        fallback_path = map_path.parent / "empty_keepout.yaml"
        keepout_path = fallback_path if fallback_path.exists() else None

    if keepout_path is None:
        return

    KEEPOUT_SELECTION_FILE.parent.mkdir(parents=True, exist_ok=True)
    try:
        KEEPOUT_SELECTION_FILE.write_text(str(keepout_path.resolve()) + "\n", encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"Gagal menyimpan keepout mask pilihan: {exc}") from exc

    if ROS_NODE is not None:
        ROS_NODE.get_logger().info(f"Keepout mask dipilih: {keepout_path}")


def save_selected_map(map_path: Path):
    MAP_SELECTION_FILE.parent.mkdir(parents=True, exist_ok=True)
    try:
        MAP_SELECTION_FILE.write_text(str(map_path) + "\n", encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"Gagal menyimpan map pilihan: {exc}") from exc

    save_selected_keepout_for_map(map_path)

    if ROS_NODE is not None:
        ROS_NODE.get_logger().info(f"Map localization dipilih: {map_path}")


def sanitize_mapping_file_stem(raw_name: str) -> str:
    name = str(raw_name or "").strip()
    for suffix in (".yaml", ".pgm", ".posegraph", ".data"):
        if name.lower().endswith(suffix):
            name = name[: -len(suffix)]
            break
    if Path(name).name != name or "\\" in name:
        raise ValueError("Nama file tidak boleh berisi path/folder.")
    name = name.strip()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,79}", name):
        raise ValueError("Nama file hanya boleh huruf, angka, titik, underscore, dan dash.")
    return name


def get_mapping_save_dir() -> Path:
    maps_dir = find_maps_folder()
    if maps_dir is not None:
        return maps_dir
    MAPPING_SAVE_DIR.mkdir(parents=True, exist_ok=True)
    return MAPPING_SAVE_DIR


def parse_map_yaml(map_path: Path) -> dict:
    metadata = {}
    for raw_line in map_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        value = value.split("#", 1)[0].strip()
        if len(value) >= 2 and value[0] in ("'", '"') and value[-1] == value[0]:
            value = value[1:-1]
        metadata[key.strip()] = value

    if "image" not in metadata:
        raise ValueError(f"Map YAML {map_path.name} tidak punya field image.")

    image_path = Path(metadata["image"]).expanduser()
    if not image_path.is_absolute():
        image_path = map_path.parent / image_path

    try:
        resolution = float(metadata.get("resolution", "0.05"))
        origin = ast.literal_eval(metadata.get("origin", "[0, 0, 0]"))
    except (ValueError, SyntaxError) as exc:
        raise ValueError(f"Metadata map {map_path.name} tidak valid: {exc}") from exc

    if not isinstance(origin, (list, tuple)) or len(origin) < 2:
        raise ValueError(f"Origin map {map_path.name} tidak valid.")

    origin = [
        float(origin[0]),
        float(origin[1]),
        float(origin[2]) if len(origin) > 2 else 0.0,
    ]

    return {
        "yaml_path": map_path,
        "image_path": image_path,
        "resolution": resolution,
        "origin": origin,
        "mode": str(metadata.get("mode", "trinary")).strip().lower(),
        "negate": int(float(metadata.get("negate", "0"))),
        "occupied_thresh": float(metadata.get("occupied_thresh", "0.65")),
        "free_thresh": float(metadata.get("free_thresh", "0.25")),
    }


def _read_pgm_token(stream) -> str:
    token = bytearray()
    while True:
        ch = stream.read(1)
        if not ch:
            break
        if ch == b"#":
            stream.readline()
            if token:
                break
            continue
        if ch.isspace():
            if token:
                break
            continue
        token.extend(ch)
    return token.decode("ascii")


def read_pgm_image(image_path: Path) -> tuple[int, int, bytes]:
    with image_path.open("rb") as stream:
        magic = _read_pgm_token(stream)
        if magic not in ("P5", "P2"):
            raise ValueError(f"Format image {image_path.name} bukan PGM P5/P2.")
        width = int(_read_pgm_token(stream))
        height = int(_read_pgm_token(stream))
        max_value = int(_read_pgm_token(stream))
        if width <= 0 or height <= 0 or max_value <= 0 or max_value > 255:
            raise ValueError(f"Header PGM {image_path.name} tidak didukung.")

        if magic == "P5":
            pixels = stream.read(width * height)
            if len(pixels) != width * height:
                raise ValueError(f"Data PGM {image_path.name} tidak lengkap.")
            if max_value != 255:
                pixels = bytes(int(round(pixel * 255.0 / max_value)) for pixel in pixels)
            return width, height, pixels

        values = []
        while len(values) < width * height:
            token = _read_pgm_token(stream)
            if not token:
                break
            values.append(int(token))
        if len(values) != width * height:
            raise ValueError(f"Data PGM ASCII {image_path.name} tidak lengkap.")
        return width, height, bytes(
            max(0, min(255, int(round(value * 255.0 / max_value))))
            for value in values
        )


def map_pixels_to_occupancy(metadata: dict, width: int, height: int, pixels: bytes) -> bytes:
    occupied_thresh = float(metadata["occupied_thresh"])
    free_thresh = float(metadata["free_thresh"])
    negate = int(metadata["negate"])
    mode = str(metadata["mode"])
    data = bytearray(width * height)

    for image_row in range(height):
        map_row = height - 1 - image_row
        for col in range(width):
            pixel = pixels[image_row * width + col]
            color = pixel / 255.0
            occ = color if negate else 1.0 - color
            if mode == "raw":
                value = int(round(pixel * 100.0 / 255.0))
            elif occ > occupied_thresh:
                value = 100
            elif occ < free_thresh:
                value = 0
            elif mode == "scale":
                value = int(round(99.0 * (occ - free_thresh) / max(0.0001, occupied_thresh - free_thresh)))
                value = max(1, min(99, value))
            else:
                value = -1
            data[map_row * width + col] = value & 0xFF

    return bytes(data)


def load_static_map_grid(map_path: Path) -> dict:
    metadata = parse_map_yaml(map_path)
    width, height, pixels = read_pgm_image(metadata["image_path"])
    origin = metadata["origin"]
    return {
        "frame_id": "map",
        "source_frame_id": "map",
        "target_frame_id": "map",
        "transform_ok": True,
        "w": width,
        "h": height,
        "res": metadata["resolution"],
        "ox": origin[0],
        "oy": origin[1],
        "oyaw": origin[2],
        "map_name": map_path.name,
        "b64": base64.b64encode(map_pixels_to_occupancy(metadata, width, height, pixels)).decode("ascii"),
    }


def nav_annotation_path(map_path: Path) -> Path:
    return map_path.with_name(f"{map_path.stem}_nav.json")


def keepout_mask_paths(map_path: Path) -> tuple[Path, Path]:
    pgm_path = map_path.with_name(f"{map_path.stem}_keepout.pgm")
    yaml_path = map_path.with_name(f"{map_path.stem}_keepout.yaml")
    return pgm_path, yaml_path


def normalize_annotation_id(raw_id: str | None, prefix: str) -> str:
    clean_id = str(raw_id or "").strip()
    if clean_id and re.fullmatch(r"[A-Za-z0-9_-]{1,80}", clean_id):
        return clean_id
    return f"{prefix}_{uuid.uuid4().hex[:10]}"


def normalize_keepout_zone(raw_zone: dict, index: int) -> dict:
    if not isinstance(raw_zone, dict):
        raise ValueError("Restriction area harus object.")
    points = raw_zone.get("points")
    if not isinstance(points, list) or len(points) != 4:
        raise ValueError("Restriction area harus berupa rectangle 4 titik.")
    clean_points = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            raise ValueError("Titik restriction area tidak valid.")
        try:
            x = float(point[0])
            y = float(point[1])
        except (TypeError, ValueError) as exc:
            raise ValueError("Koordinat restriction area tidak valid.") from exc
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Koordinat restriction area tidak valid.")
        clean_points.append([round(x, 4), round(y, 4)])
    return {
        "id": normalize_annotation_id(raw_zone.get("id"), "zone"),
        "name": str(raw_zone.get("name") or f"Restriction {index + 1}").strip()[:80],
        "enabled": bool(raw_zone.get("enabled", True)),
        "points": clean_points,
    }


def normalize_station(raw_station: dict, index: int) -> dict:
    if not isinstance(raw_station, dict):
        raise ValueError("Station harus object.")
    try:
        x = float(raw_station.get("x"))
        y = float(raw_station.get("y"))
        theta = float(raw_station.get("theta"))
        wait_sec = float(raw_station.get("wait_sec", 0.0) or 0.0)
    except (TypeError, ValueError) as exc:
        raise ValueError("Koordinat station tidak valid.") from exc
    if not all(math.isfinite(value) for value in (x, y, theta, wait_sec)):
        raise ValueError("Koordinat station tidak valid.")
    if wait_sec < 0:
        raise ValueError("wait_sec station tidak boleh negatif.")
    return {
        "id": normalize_annotation_id(raw_station.get("id"), "station"),
        "name": str(raw_station.get("name") or f"Station {index + 1}").strip()[:80],
        "x": round(x, 4),
        "y": round(y, 4),
        "theta": round(theta, 6),
        "wait_sec": round(wait_sec, 2),
        "enabled": bool(raw_station.get("enabled", True)),
    }


def point_in_polygon(x: float, y: float, points: list[list[float]]) -> bool:
    inside = False
    count = len(points)
    j = count - 1
    for i in range(count):
        xi, yi = points[i]
        xj, yj = points[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1.0e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def write_keepout_mask(map_path: Path, zones: list[dict]) -> dict:
    metadata = parse_map_yaml(map_path)
    width, height, _pixels = read_pgm_image(metadata["image_path"])
    resolution = float(metadata["resolution"])
    ox, oy, _oyaw = metadata["origin"]
    pgm_path, yaml_path = keepout_mask_paths(map_path)
    mask = bytearray([255] * (width * height))

    for zone in zones:
        if not zone.get("enabled", True):
            continue
        points = zone["points"]
        min_x = min(point[0] for point in points)
        max_x = max(point[0] for point in points)
        min_y = min(point[1] for point in points)
        max_y = max(point[1] for point in points)
        col_start = max(0, int(math.floor((min_x - ox) / resolution)) - 1)
        col_end = min(width - 1, int(math.ceil((max_x - ox) / resolution)) + 1)
        row_start = max(0, int(math.floor((min_y - oy) / resolution)) - 1)
        row_end = min(height - 1, int(math.ceil((max_y - oy) / resolution)) + 1)

        for grid_row in range(row_start, row_end + 1):
            wy = oy + (grid_row + 0.5) * resolution
            image_row = height - 1 - grid_row
            for col in range(col_start, col_end + 1):
                wx = ox + (col + 0.5) * resolution
                if point_in_polygon(wx, wy, points):
                    mask[image_row * width + col] = 0

    with pgm_path.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(mask)

    origin = metadata["origin"]
    yaml_text = "\n".join([
        f"image: {pgm_path.name}",
        "mode: trinary",
        f"resolution: {resolution}",
        f"origin: [{origin[0]}, {origin[1]}, 0]",
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.25",
        "",
    ])
    yaml_path.write_text(yaml_text, encoding="utf-8")
    KEEPOUT_SELECTION_FILE.parent.mkdir(parents=True, exist_ok=True)
    KEEPOUT_SELECTION_FILE.write_text(str(yaml_path.resolve()) + "\n", encoding="utf-8")

    return {
        "pgm_path": str(pgm_path),
        "yaml_path": str(yaml_path),
        "yaml_name": yaml_path.name,
        "width": width,
        "height": height,
    }


def read_nav_annotations(map_name: str) -> dict:
    map_path = resolve_map_path(map_name)
    annotations_path = nav_annotation_path(map_path)
    if not annotations_path.exists():
        pgm_path, yaml_path = keepout_mask_paths(map_path)
        return {
            "map_name": map_path.name,
            "zones": [],
            "stations": [],
            "annotation_path": str(annotations_path),
            "keepout_yaml": yaml_path.name if yaml_path.exists() else "",
            "keepout_pgm": pgm_path.name if pgm_path.exists() else "",
        }

    try:
        data = json.loads(annotations_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Gagal membaca annotation {annotations_path.name}: {exc}") from exc

    zones = [
        normalize_keepout_zone(zone, index)
        for index, zone in enumerate(data.get("zones", []))
    ]
    stations = [
        normalize_station(station, index)
        for index, station in enumerate(data.get("stations", []))
    ]
    pgm_path, yaml_path = keepout_mask_paths(map_path)
    return {
        "map_name": map_path.name,
        "zones": zones,
        "stations": stations,
        "annotation_path": str(annotations_path),
        "keepout_yaml": yaml_path.name if yaml_path.exists() else "",
        "keepout_pgm": pgm_path.name if pgm_path.exists() else "",
    }


def save_nav_annotations(map_name: str, zones: list[dict], stations: list[dict]) -> dict:
    map_path = resolve_map_path(map_name)
    clean_zones = [
        normalize_keepout_zone(zone, index)
        for index, zone in enumerate(zones or [])
    ]
    clean_stations = [
        normalize_station(station, index)
        for index, station in enumerate(stations or [])
    ]
    annotations_path = nav_annotation_path(map_path)
    payload = {
        "map_name": map_path.name,
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "zones": clean_zones,
        "stations": clean_stations,
    }
    annotations_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    mask = write_keepout_mask(map_path, clean_zones)
    return {
        "ok": True,
        "message": "Restriction area dan station tersimpan. Restart Nav2 untuk menerapkan keepout.",
        "map_name": map_path.name,
        "zones": clean_zones,
        "stations": clean_stations,
        "annotation_path": str(annotations_path),
        "keepout": mask,
        "restart_required": True,
    }


def parse_slam_toolbox_result(output: str) -> int | None:
    match = re.search(r"\bresult\s*[:=]\s*(\d+)", output or "")
    if not match:
        return None
    return int(match.group(1))


def run_terminal_command(command_args: list[str], cwd: Path, timeout_sec: float = 30.0) -> dict:
    command_text = shlex.join(command_args)
    completed = subprocess.run(
        ["bash", "-ic", command_text],
        cwd=str(cwd),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout_sec,
    )
    return {
        "command": command_text,
        "returncode": completed.returncode,
        "output": completed.stdout or "",
        "result_code": parse_slam_toolbox_result(completed.stdout or ""),
    }


def save_slam_toolbox_map(raw_name: str) -> dict:
    stem = sanitize_mapping_file_stem(raw_name)
    maps_dir = get_mapping_save_dir()
    file_prefix = str((maps_dir / stem).resolve())

    save_args = [
        "ros2",
        "service",
        "call",
        "/slam_toolbox/save_map",
        "slam_toolbox/srv/SaveMap",
        f"{{name: {{data: '{file_prefix}'}}}}",
    ]
    serialize_args = [
        "ros2",
        "service",
        "call",
        "/slam_toolbox/serialize_map",
        "slam_toolbox/srv/SerializePoseGraph",
        f"{{filename: '{file_prefix}'}}",
    ]

    steps = []
    for label, command_args in (("save_map", save_args), ("serialize_map", serialize_args)):
        try:
            step = run_terminal_command(command_args, maps_dir)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"{label} timeout: {exc}") from exc
        except OSError as exc:
            raise RuntimeError(f"{label} gagal dijalankan: {exc}") from exc

        step["step"] = label
        steps.append(step)
        result_code = step.get("result_code")
        if step["returncode"] != 0:
            raise RuntimeError(f"{label} gagal. Output: {step['output'].strip() or step['command']}")
        if result_code is not None and result_code != 0:
            raise RuntimeError(f"{label} mengembalikan result={result_code}. Output: {step['output'].strip()}")

    yaml_path = maps_dir / f"{stem}.yaml"
    pgm_path = maps_dir / f"{stem}.pgm"
    missing_files = [str(path) for path in (yaml_path, pgm_path) if not path.exists()]
    if missing_files:
        raise RuntimeError("save_map selesai tapi file belum ditemukan: " + ", ".join(missing_files))

    with MAP_SELECTION_LOCK:
        save_selected_map(yaml_path)

    return {
        "ok": True,
        "message": f"Map {stem}.yaml/.pgm tersimpan dan pose graph diserialize.",
        "map_name": f"{stem}.yaml",
        "stem": stem,
        "maps_dir": str(maps_dir),
        "yaml_path": str(yaml_path),
        "pgm_path": str(pgm_path),
        "steps": steps,
    }


class LocalizationRequest(BaseModel):
    map_name: str


class MappingSaveRequest(BaseModel):
    map_name: str


class NavAnnotationsRequest(BaseModel):
    map_name: str
    zones: list[dict] = []
    stations: list[dict] = []

# Mount folder static
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

# ===================== HTTP ROUTE =====================
@app.get("/")
async def index():
    return FileResponse(
        STATIC_DIR / MAIN_UI_FILE,
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


@app.get("/api/maps/grid")
async def get_map_grid(map_name: str):
    try:
        map_path = resolve_map_path(map_name)
        return load_static_map_grid(map_path)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"Gagal membaca map: {exc}") from exc


@app.get("/api/nav_annotations")
async def get_nav_annotations(map_name: str):
    try:
        return read_nav_annotations(map_name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.post("/api/nav_annotations")
async def post_nav_annotations(request: NavAnnotationsRequest):
    try:
        return await asyncio.to_thread(
            save_nav_annotations,
            request.map_name,
            request.zones,
            request.stations,
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"Gagal menyimpan annotation: {exc}") from exc


@app.get("/healthz")
async def healthz():
    return {
        "ok": True,
        "ros_ready": ROS_NODE is not None,
        "static_dir": str(STATIC_DIR),
        "main_ui_file": MAIN_UI_FILE,
    }


@app.get("/api/topics")
async def get_topics():
    if ROS_NODE is None:
        return {
            "topics": [],
            "max_echo_slots": 2,
            "ros_ready": False,
        }

    return {
        "topics": ROS_NODE.list_ros_topics(),
        "max_echo_slots": 2,
        "ros_ready": True,
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


@app.post("/api/mapping/save")
async def save_mapping_map(request: MappingSaveRequest):
    if ROS_NODE is None:
        raise HTTPException(status_code=503, detail="ROS node belum siap.")

    def locked_save():
        with MAPPING_SAVE_LOCK:
            return save_slam_toolbox_map(request.map_name)

    try:
        result = await asyncio.to_thread(locked_save)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc

    if ROS_NODE is not None:
        ROS_NODE.get_logger().info(result["message"])

    return result


@app.websocket("/ws/topics")
async def ws_topics(websocket: WebSocket):
    await websocket.accept()
    subscriptions = []
    slot_states = {}
    state_lock = threading.Lock()
    echo_rate_hz = 5
    min_convert_interval = 1.0 / echo_rate_hz

    def clear_subscriptions():
        nonlocal subscriptions
        if ROS_NODE is not None:
            for subscription in subscriptions:
                try:
                    ROS_NODE.destroy_echo_subscription(subscription)
                except Exception as exc:
                    ROS_NODE.get_logger().warn(f"Gagal melepas topic echo subscription: {exc}")
        subscriptions = []
        with state_lock:
            slot_states.clear()

    def make_echo_callback(slot: int):
        def cb(msg):
            now = time.monotonic()
            with state_lock:
                state = slot_states.get(slot)
                if state is None:
                    return
                if now - state["last_convert"] < min_convert_interval:
                    return
                state["last_convert"] = now

            data = ros_value_to_bounded_data(msg)

            with state_lock:
                state = slot_states.get(slot)
                if state is None:
                    return
                state["count"] += 1
                state["stamp"] = time.strftime("%H:%M:%S")
                state["data"] = data
                state["dirty"] = True

        return cb

    async def subscribe_topics(topics):
        if ROS_NODE is None:
            await websocket.send_text(json.dumps({
                "type": "error",
                "detail": "ROS node belum siap.",
            }))
            return

        clean_topics = []
        for raw_topic in topics:
            clean_topic = normalize_topic_name(raw_topic)
            if clean_topic and clean_topic not in clean_topics:
                clean_topics.append(clean_topic)

        if len(clean_topics) > 2:
            await websocket.send_text(json.dumps({
                "type": "error",
                "detail": "Echo dibatasi maksimal 2 ROS topic.",
            }))
            return

        clear_subscriptions()
        slots = []

        try:
            for slot, topic_name in enumerate(clean_topics, start=1):
                with state_lock:
                    slot_states[slot] = {
                        "slot": slot,
                        "topic": topic_name,
                        "msg_type": "",
                        "stamp": "waiting",
                        "count": 0,
                        "data": {"status": "waiting for message"},
                        "dirty": True,
                        "last_convert": 0.0,
                    }

                subscription, msg_type = ROS_NODE.create_echo_subscription(
                    topic_name,
                    make_echo_callback(slot),
                )
                subscriptions.append(subscription)

                with state_lock:
                    slot_states[slot]["msg_type"] = msg_type

                slots.append({
                    "slot": slot,
                    "topic": topic_name,
                    "msg_type": msg_type,
                })
        except Exception as exc:
            clear_subscriptions()
            await websocket.send_text(json.dumps({
                "type": "error",
                "detail": str(exc),
            }))
            return

        await websocket.send_text(json.dumps({
            "type": "subscribed",
            "slots": slots,
            "rate_hz": echo_rate_hz,
        }))

    try:
        while True:
            try:
                msg = await asyncio.wait_for(websocket.receive_text(), timeout=0.2)
                data = json.loads(msg)

                if data.get("type") == "subscribe":
                    topics = data.get("topics", [])
                    if not isinstance(topics, list):
                        raise ValueError("Payload topics harus list")
                    await subscribe_topics(topics)
                elif data.get("type") == "clear":
                    clear_subscriptions()
                    await websocket.send_text(json.dumps({
                        "type": "subscribed",
                        "slots": [],
                        "rate_hz": echo_rate_hz,
                    }))
            except asyncio.TimeoutError:
                pass
            except (json.JSONDecodeError, ValueError) as exc:
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "detail": str(exc),
                }))

            updates = []
            with state_lock:
                for state in slot_states.values():
                    if not state["dirty"]:
                        continue
                    updates.append({
                        "slot": state["slot"],
                        "topic": state["topic"],
                        "msg_type": state["msg_type"],
                        "stamp": state["stamp"],
                        "count": state["count"],
                        "data": state["data"],
                    })
                    state["dirty"] = False

            if updates:
                await websocket.send_text(json.dumps({
                    "type": "echo",
                    "updates": updates,
                }))

    except WebSocketDisconnect:
        pass
    finally:
        clear_subscriptions()


# ===================== WEBSOCKET HANDLER =====================
@app.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()
    last_cmd_time = 0.0
    last_v, last_w = 0.0, 0.0
    last_telemetry_time = 0.0
    telemetry_interval = 0.1

    if ROS_NODE:
        ROS_NODE.map_dirty = True
        ROS_NODE.local_costmap_dirty = True
        ROS_NODE.global_costmap_dirty = True
        ROS_NODE.path_dirty = True
        ROS_NODE.lidar_scan_dirty = True

    while True:
        try:
            now = asyncio.get_event_loop().time()

            # kirim telemetry ringan pada 10 Hz
            if ROS_NODE and now - last_telemetry_time >= telemetry_interval:
                last_telemetry_time = now
                amcl_pose = ROS_NODE.amcl_pose
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
                    "amcl_x": amcl_pose[0] if amcl_pose else None,
                    "amcl_y": amcl_pose[1] if amcl_pose else None,
                    "amcl_theta": amcl_pose[2] if amcl_pose else None,
                    "launches": ROS_NODE.get_launch_statuses(),
                    "io_inputs":  ROS_NODE.io_inputs_byte,
                    "io_outputs": ROS_NODE.io_outputs_byte,
                    "lidar_motor": ROS_NODE.lidar_motor_enabled,
                    "drive_mode": ROS_NODE.drive_mode,
                    "nav_goal_status": ROS_NODE.get_nav_goal_status(),
                    "mission_status": ROS_NODE.get_station_mission_status(),
                }))

            if ROS_NODE and getattr(ROS_NODE, 'map_dirty', False):
                ROS_NODE.map_dirty = False
                await websocket.send_text(json.dumps({"type": "nav_map", "data": ROS_NODE.map_data}))

            if ROS_NODE and getattr(ROS_NODE, 'local_costmap_dirty', False):
                ROS_NODE.local_costmap_dirty = False
                await websocket.send_text(json.dumps({"type": "nav_local_costmap", "data": ROS_NODE.local_costmap_data}))

            if ROS_NODE and getattr(ROS_NODE, 'global_costmap_dirty', False):
                ROS_NODE.global_costmap_dirty = False
                await websocket.send_text(json.dumps({"type": "nav_global_costmap", "data": ROS_NODE.global_costmap_data}))

            if ROS_NODE and getattr(ROS_NODE, 'path_dirty', False):
                ROS_NODE.path_dirty = False
                await websocket.send_text(json.dumps({"type": "nav_path", "data": ROS_NODE.path_data}))

            if ROS_NODE and getattr(ROS_NODE, 'lidar_scan_dirty', False):
                ROS_NODE.lidar_scan_dirty = False
                await websocket.send_text(json.dumps({
                    "type": "nav_lidar_scan",
                    "data": ROS_NODE.lidar_scan_data,
                }))

            # baca perintah web
            msg = await asyncio.wait_for(websocket.receive_text(), timeout=0.02)
            data = json.loads(msg)

            if ROS_NODE is None:
                continue

            if data.get("type") == "cmd_vel":
                v = float(data["linear"])
                w = float(data["angular"])
                if ROS_NODE.send_cmd_vel(v, w):
                    last_cmd_time = asyncio.get_event_loop().time()
                    last_v, last_w = v, w

            elif data.get("type") == "mode":
                ROS_NODE.set_drive_mode(data.get("value"))
                last_v, last_w = 0.0, 0.0

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
            elif data.get("type") == "goal_pose":
                result = ROS_NODE.send_goal(data["x"], data["y"], data["theta"])
                result["type"] = "goal_pose_ack"
                result["drive_mode"] = ROS_NODE.drive_mode
                result["nav_goal_status"] = ROS_NODE.get_nav_goal_status()
                result["mission_status"] = ROS_NODE.get_station_mission_status()
                await websocket.send_text(json.dumps(result))
            elif data.get("type") in ("initial_pose", "init_pose", "set_initial_pose"):
                result = ROS_NODE.set_initial_pose(data["x"], data["y"], data["theta"])
                result["type"] = "initial_pose_ack"
                await websocket.send_text(json.dumps(result))
            elif data.get("type") == "mission_queue_start":
                result = ROS_NODE.start_station_queue(data.get("stations", []))
                result["type"] = "mission_queue_ack"
                result["mission_status"] = ROS_NODE.get_station_mission_status()
                await websocket.send_text(json.dumps(result))
            elif data.get("type") == "mission_queue_cancel":
                result = ROS_NODE.cancel_station_queue()
                result["type"] = "mission_queue_ack"
                result["mission_status"] = ROS_NODE.get_station_mission_status()
                await websocket.send_text(json.dumps(result))
            elif data.get("type") == "mission_status":
                await websocket.send_text(json.dumps({
                    "type": "mission_status",
                    "mission_status": ROS_NODE.get_station_mission_status(),
                }))
            elif data.get("type") == "launch_start":
                result = ROS_NODE.start_launch_preset(data.get("name"))
                result["type"] = "launch_result"
                result["launches"] = ROS_NODE.get_launch_statuses()
                await websocket.send_text(json.dumps(result))
            elif data.get("type") == "launch_stop":
                result = ROS_NODE.stop_launch_preset(data.get("name"))
                result["type"] = "launch_result"
                result["launches"] = ROS_NODE.get_launch_statuses()
                await websocket.send_text(json.dumps(result))
            elif data.get("type") == "launch_status":
                await websocket.send_text(json.dumps({
                    "type": "launch_status",
                    "launches": ROS_NODE.get_launch_statuses(),
                }))

        except asyncio.TimeoutError:
            # tidak ada command baru dalam 0.1s → stop
            now = asyncio.get_event_loop().time()
            if ROS_NODE and ROS_NODE.drive_mode == "manual" and now - last_cmd_time > 0.1 and (last_v != 0.0 or last_w != 0.0):
                ROS_NODE.send_cmd_vel(0.0, 0.0)
                last_v, last_w = 0.0, 0.0
            continue

        except WebSocketDisconnect:
            if ROS_NODE and ROS_NODE.drive_mode == "manual":
                ROS_NODE.send_cmd_vel(0.0, 0.0)
            break


# ===================== ROS2 THREAD =====================
def ros_thread():
    global ROS_NODE
    try:
        if SignalHandlerOptions is None:
            rclpy.init()
        else:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        ROS_NODE = AryaWebNode()
        rclpy.spin(ROS_NODE)
    except Exception as exc:
        print(f"[ERROR] ARYA ROS thread gagal: {exc}", flush=True)
        raise
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def start_ros_thread():
    global ROS_THREAD_STARTED
    if ROS_THREAD_STARTED:
        return
    ROS_THREAD_STARTED = True
    print("[INFO] ARYA ROS thread starting", flush=True)
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()


# ===================== MAIN ENTRY =====================
def main():
    print("[INFO] ARYA web server starting on http://0.0.0.0:8000", flush=True)
    ros_start_timer = threading.Timer(0.5, start_ros_thread)
    ros_start_timer.daemon = True
    ros_start_timer.start()
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    except Exception as exc:
        print(f"[ERROR] ARYA web server gagal start di port 8000: {exc}", flush=True)
        raise


if __name__ == "__main__":
    main()
