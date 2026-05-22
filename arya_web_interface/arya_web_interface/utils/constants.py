"""Configuration constants for the web interface."""

from action_msgs.msg import GoalStatus

# Navigation Goal Status Mapping
NAV_GOAL_STATUS_LABELS = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}

# Launch Presets Configuration
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

# Known ROS Topic Type Hints
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

# Add dynamic IO channels
for _channel in range(1, 9):
    KNOWN_ROS_TOPIC_TYPES[f"/io/di{_channel}"] = ["std_msgs/msg/Bool"]
    KNOWN_ROS_TOPIC_TYPES[f"/io/do{_channel}"] = ["std_msgs/msg/Bool"]

# Data Limits
MAX_ECHO_SEQUENCE_ITEMS = 16
MAX_ECHO_STRING_CHARS = 1200
MAX_ECHO_DEPTH = 3
MAX_LIDAR_POINTS = 720
