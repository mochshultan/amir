"""ROS Node implementation with thread-safe state management."""

import threading
import time
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from std_msgs.msg import Float32MultiArray, UInt8, String
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Empty
from pathlib import Path
from ament_index_python.packages import get_package_prefix

try:
    from lifecycle_msgs.srv import GetState
except Exception:
    GetState = None

from ..utils.constants import (
    NAV_GOAL_STATUS_LABELS,
    LAUNCH_PRESETS,
    MAX_LIDAR_POINTS,
)


class AryaWebNode(Node):
    """
    ROS2 Node for web interface control of autonomous mobile robot.
    
    Thread-safe by design with locks protecting shared state:
    - nav_goal_lock: Navigation goal state (current goal, seq, contexts)
    - mission_lock: Station queue mission state
    - drive_mode_lock: Drive mode selection
    - launch_lock: Launch process tracking
    - topic_echo_lock: Dynamic topic subscriptions
    - imu_restart_lock: IMU process restart
    - localization_reset_lock: Localization reset operations
    """
    
    def __init__(self):
        super().__init__('arya_web_node')
        self.get_logger().info("Initializing AryaWebNode...")
        
        # Node parameters
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
        
        # Load parameters
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
        
        # Thread synchronization
        self.imu_restart_lock = threading.Lock()
        self.localization_reset_lock = threading.Lock()
        self.topic_echo_lock = threading.Lock()
        self.drive_mode_lock = threading.Lock()
        self.launch_lock = threading.Lock()
        
        # Launch process state
        self.launch_processes = {}
        self.launch_log_dir = Path.home() / ".arya_amr" / "logs"
        
        # TF2 buffer for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_lidar_tf_warn_time = 0.0
        self.last_grid_tf_warn_times = {}
        
        # Robot state
        self.telemetry = [0, 0, 0, 0, 0, 0]
        self.odom = [0, 0, 0]
        self.amcl_pose = None
        self.io_inputs_byte = 0
        self.io_outputs_byte = 0
        
        # Navigation state
        self.nav_goal_lock = threading.Lock()
        self.nav_goal_seq = 0
        self.current_nav_goal_handle = None
        self.nav_goal_contexts = {}
        self.nav_goal_status = {
            "state": "idle",
            "message": "Belum ada goal dari web.",
            "seq": 0,
        }
        
        # Mission queue state
        self.mission_lock = threading.Lock()
        self.mission_seq = 0
        self.station_mission_status = {
            "state": "idle",
            "message": "Belum ada station mission.",
            "mode": "idle",
            "mission_id": 0,
            "current_index": -1,
            "total": 0,
            "station": None,
        }
        
        # Map data
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
        
        # QoS profiles
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        qos_transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        qos_best_effort = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscriptions
        self.sub_telemetry = self.create_subscription(
            Float32MultiArray, 'telemetry', self._cb_telemetry, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self._cb_odom, 10
        )
        self.sub_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._cb_amcl_pose, 10
        )
        self.sub_io_inputs = self.create_subscription(
            UInt8, 'io/inputs_raw', self._cb_io_inputs, 10
        )
        self.sub_io_outputs = self.create_subscription(
            UInt8, 'io/outputs_raw', self._cb_io_outputs, 10
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self._cb_map, qos_transient
        )
        self.sub_local_costmap = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._cb_local_costmap, qos_best_effort
        )
        self.sub_global_costmap = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._cb_global_costmap, qos_transient
        )
        self.sub_path = self.create_subscription(
            NavPath, '/plan', self._cb_path, 1
        )
        self.sub_lidar_scan = self.create_subscription(
            LaserScan, '/scan', self._cb_lidar_scan, qos_best_effort
        )
        
        # Publishers
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, qos_reliable)
        self.pub_reset_odom = self.create_publisher(Float32MultiArray, 'reset_odom', qos_reliable)
        self.pub_reset_encoder = self.create_publisher(Float32MultiArray, 'reset_encoder', qos_reliable)
        self.pub_io_cmd = self.create_publisher(String, 'io/cmd_single', 10)
        self.pub_io_mask = self.create_publisher(UInt8, 'io/cmd_mask', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        
        # Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lidar_start_client = self.create_client(Empty, 'start_motor')
        self.lidar_stop_client = self.create_client(Empty, 'stop_motor')
        
        # Lifecycle state clients
        self.lifecycle_state_clients = {}
        if GetState is not None:
            for node_name in ("bt_navigator", "planner_server", "controller_server"):
                self.lifecycle_state_clients[node_name] = self.create_client(
                    GetState,
                    f"/{node_name}/get_state",
                )
        
        self.get_logger().info(
            f"AryaWebNode initialized. cmd_vel: {self.cmd_vel_topic}, mode: {self.drive_mode}"
        )
    
    # Callback methods (all prefixed with _cb_ to indicate they're callbacks)
    def _cb_telemetry(self, msg):
        self.telemetry = list(msg.data)
    
    def _cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        theta = math.atan2(math.sin(theta), math.cos(theta))
        self.odom = [x, y, theta]
    
    def _cb_amcl_pose(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        theta = math.atan2(math.sin(theta), math.cos(theta))
        self.amcl_pose = [x, y, theta]
    
    def _cb_io_inputs(self, msg):
        self.io_inputs_byte = int(msg.data)
    
    def _cb_io_outputs(self, msg):
        self.io_outputs_byte = int(msg.data)
    
    def _cb_map(self, msg):
        self.map_data = self._parse_grid(msg)
        self.map_dirty = True
    
    def _cb_local_costmap(self, msg):
        self.local_costmap_data = self._parse_grid(msg)
        self.local_costmap_dirty = True
    
    def _cb_global_costmap(self, msg):
        self.global_costmap_data = self._parse_grid(msg)
        self.global_costmap_dirty = True
    
    def _cb_path(self, msg):
        self.path_data = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        self.path_dirty = True
    
    def _cb_lidar_scan(self, msg: LaserScan):
        """Process LiDAR scan and transform to base frame."""
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
            yaw = self._yaw_from_quaternion(transform.transform.rotation)
            transformed_frame = target_frame
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_lidar_tf_warn_time > 5.0:
                self.get_logger().warn(
                    f"TF {target_frame} <- {source_frame} not available: {exc}"
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
            points.append([round(bx, 3), round(by, 3)])
        
        self.lidar_scan_data = {
            "frame_id": transformed_frame,
            "source_frame_id": source_frame,
            "points": points,
        }
        self.lidar_scan_dirty = True
    
    @staticmethod
    def _yaw_from_quaternion(q):
        """Extract yaw angle from quaternion."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        return math.atan2(math.sin(theta), math.cos(theta))
    
    def _parse_grid(self, msg):
        """Parse OccupancyGrid message with TF transformation."""
        import base64
        source_frame = (msg.header.frame_id or "").strip().lstrip("/") or "map"
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        oyaw = self._yaw_from_quaternion(msg.info.origin.orientation)
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
        """Transform grid origin from source frame to map frame."""
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
            tyaw = self._yaw_from_quaternion(transform.transform.rotation)
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
                    f"TF map <- {source_frame} not available: {exc}"
                )
                self.last_grid_tf_warn_times[source_frame] = now
            return {
                "frame_id": source_frame,
                "transform_ok": False,
                "ox": ox,
                "oy": oy,
                "oyaw": oyaw,
            }
