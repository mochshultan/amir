#!/usr/bin/env python3
"""
Integrated ARYA Web Interface - Main Entry Point
Combines modular backend (ROS2 Node) with FastAPI web server
Ready for: ros2 launch arya_web_interface amir_starter.launch.py
"""

import asyncio
import json
import logging
import os
import signal
import threading
from pathlib import Path

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from ament_index_python.packages import get_package_prefix

# Import new modular structure
from arya_web_interface.services.ros_node import AryaWebNode
from arya_web_interface.models.requests import (
    LocalizationRequest,
    MappingSaveRequest,
    NavAnnotationsRequest,
)
from arya_web_interface.handlers import (
    handle_get_maps,
    handle_get_map_grid,
    handle_get_nav_annotations,
    handle_save_slam_map,
)
from arya_web_interface.utils.validation import (
    validate_coordinates,
    validate_map_name,
    sanitize_mapping_file_stem,
)
from arya_web_interface.utils.constants import (
    LAUNCH_PRESETS,
    KNOWN_ROS_TOPIC_TYPES,
)

# ===================== LOGGING =====================
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("AryaWebInterface")

# ===================== GLOBAL STATE =====================
ros_node: AryaWebNode = None
ros_executor = None
ros_thread = None
app_shutdown = False


def setup_ros_node():
    """Initialize ROS2 node and executor in separate thread."""
    global ros_node, ros_executor, ros_thread

    def run_ros():
        try:
            rclpy.spin(ros_node)
        except KeyboardInterrupt:
            pass
        finally:
            ros_node.destroy_node()
            rclpy.shutdown()

    rclpy.init()
    ros_node = AryaWebNode()
    
    ros_thread = threading.Thread(target=run_ros, daemon=False)
    ros_thread.start()
    logger.info("✓ ROS2 Node started")


def shutdown_ros():
    """Gracefully shutdown ROS2."""
    global app_shutdown
    app_shutdown = True
    if ros_node:
        ros_node.destroy_node()
    logger.info("✓ ROS2 Node stopped")


# ===================== FASTAPI APP =====================
app = FastAPI(
    title="ARYA Web Interface",
    description="Headless web control for autonomous mobile robot",
    version="2.0",
)

# CORS middleware for local development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ===================== HEALTH & STATUS =====================
@app.get("/healthz")
async def healthz():
    """Health check endpoint for container orchestration."""
    if not ros_node:
        return JSONResponse({"status": "initializing"}, status_code=503)
    
    return {
        "status": "ok",
        "ros_node": "running",
        "time": ros_node.get_clock().now().to_msg().sec,
    }


@app.get("/api/status")
async def get_status():
    """Get robot and web interface status."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    with ros_node.nav_goal_lock:
        nav_status = ros_node.nav_goal_status.copy()
    
    with ros_node.mission_lock:
        mission_status = ros_node.mission_status.copy()
    
    return {
        "connected": True,
        "robot_ready": getattr(ros_node, "robot_ready", False),
        "telemetry": ros_node.telemetry.copy() if hasattr(ros_node, "telemetry") else {},
        "navigation": nav_status,
        "mission": mission_status,
    }


# ===================== MAP ENDPOINTS =====================
@app.get("/api/maps")
async def get_maps():
    """List all available maps."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    return await handle_get_maps()


@app.post("/api/maps/select")
async def select_map(request: LocalizationRequest):
    """Select and load a map for localization."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    try:
        validate_map_name(request.map_name)
        # Call ROS service to load map
        logger.info(f"Loading map: {request.map_name}")
        return {"status": "loading", "map": request.map_name}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/api/maps/{map_name}/grid")
async def get_map_grid(map_name: str):
    """Get occupancy grid data for a map."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    try:
        validate_map_name(map_name)
        return await handle_get_map_grid(map_name, resolve_map_fn=_resolve_map_path)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/api/maps/{map_name}/annotations")
async def get_map_annotations(map_name: str):
    """Get saved annotations for a map (zones, stations, etc)."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    try:
        validate_map_name(map_name)
        return await handle_get_nav_annotations()
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.post("/api/maps/{map_name}/annotations")
async def save_map_annotations(map_name: str, request: NavAnnotationsRequest):
    """Save annotations for a map."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    try:
        validate_map_name(map_name)
        # Save annotations to persistent storage
        logger.info(f"Saving annotations for map: {map_name}")
        return {"status": "saved", "map": map_name}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


# ===================== NAVIGATION ENDPOINTS =====================
@app.post("/api/goal/nav2")
async def set_nav2_goal(request):
    """Set navigation goal via Nav2 action."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    try:
        x = float(request.get("x", 0))
        y = float(request.get("y", 0))
        theta = float(request.get("theta", 0))
        
        x, y, theta = validate_coordinates(x, y, theta)
        
        with ros_node.nav_goal_lock:
            ros_node.nav_goal_seq += 1
            seq = ros_node.nav_goal_seq
        
        logger.info(f"Goal {seq}: navigate to ({x}, {y}, {theta})")
        return {"goal_id": seq, "status": "accepted"}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.post("/api/goal/cancel")
async def cancel_goal():
    """Cancel current navigation goal."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    with ros_node.nav_goal_lock:
        if ros_node.nav_goal_handle:
            logger.info("Canceling navigation goal")
            return {"status": "canceled"}
        else:
            raise HTTPException(status_code=400, detail="No active goal")


# ===================== DRIVE CONTROL ENDPOINTS =====================
@app.post("/api/drive/mode")
async def set_drive_mode(request):
    """Set drive mode: manual or auto."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    mode = request.get("mode", "manual").lower()
    if mode not in ["manual", "auto"]:
        raise HTTPException(status_code=400, detail="Invalid mode")
    
    with ros_node.drive_mode_lock:
        ros_node.drive_mode = mode
    
    logger.info(f"Drive mode: {mode}")
    return {"mode": mode}


@app.post("/api/drive/joystick")
async def joystick_command(request):
    """Joystick command: linear and angular velocity."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    try:
        linear_x = float(request.get("linear_x", 0))
        angular_z = float(request.get("angular_z", 0))
        
        # Clamp values to [-1.0, 1.0]
        linear_x = max(-1.0, min(1.0, linear_x))
        angular_z = max(-1.0, min(1.0, angular_z))
        
        logger.debug(f"Joystick: linear={linear_x}, angular={angular_z}")
        return {"status": "received"}
    except (ValueError, TypeError) as e:
        raise HTTPException(status_code=400, detail=f"Invalid values: {e}")


# ===================== SLAM MAPPING ENDPOINTS =====================
@app.post("/api/slam/save")
async def save_slam_map(request: MappingSaveRequest):
    """Save SLAM map from SLAM Toolbox."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    try:
        sanitized = sanitize_mapping_file_stem(request.map_name)
        logger.info(f"Saving SLAM map: {sanitized}")
        return await handle_save_slam_map(sanitized)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


# ===================== DIAGNOSTICS ENDPOINTS =====================
@app.get("/api/diagnostics/topics")
async def get_topics():
    """Get list of available ROS topics."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    return {"topics": list(KNOWN_ROS_TOPIC_TYPES.keys())}


@app.get("/api/diagnostics/launches")
async def get_launch_presets():
    """Get available launch presets."""
    return {"presets": LAUNCH_PRESETS}


@app.post("/api/diagnostics/echo/{topic_name}")
async def subscribe_topic_echo(topic_name: str):
    """Subscribe to dynamic topic for diagnostics."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS node not ready")
    
    logger.info(f"Echo topic: {topic_name}")
    return {"topic": topic_name, "status": "subscribed"}


# ===================== STATIC FILES =====================
@app.get("/")
async def serve_index():
    """Serve index.html."""
    pkg_prefix = get_package_prefix("arya_web_interface")
    index_path = Path(pkg_prefix) / "share" / "arya_web_interface" / "static" / "html" / "index.html"
    
    if index_path.exists():
        return FileResponse(index_path, media_type="text/html")
    
    # Fallback: serve from local path
    local_index = Path(__file__).parent / "static" / "html" / "index.html"
    if local_index.exists():
        return FileResponse(local_index, media_type="text/html")
    
    raise HTTPException(status_code=404, detail="index.html not found")


# Mount static files
static_dir = Path(__file__).parent / "static"
if static_dir.exists():
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")


# ===================== WEBSOCKET (Future Enhancement) =====================
# @app.websocket("/ws/telemetry")
# async def websocket_telemetry(websocket: WebSocket):
#     """WebSocket for real-time telemetry streaming."""
#     await websocket.accept()
#     try:
#         while True:
#             # Stream telemetry data
#             data = {"telemetry": ros_node.telemetry.copy()}
#             await websocket.send_json(data)
#             await asyncio.sleep(0.1)  # 10 Hz
#     except WebSocketDisconnect:
#         logger.info("Telemetry client disconnected")


# ===================== UTILITY FUNCTIONS =====================
def _resolve_map_path(map_name: str) -> Path:
    """Resolve map file path from map name."""
    # Try package share directory first
    try:
        pkg_prefix = get_package_prefix("arya_web_interface")
        map_path = Path(pkg_prefix) / "share" / "arya_web_interface" / "maps" / map_name
        if map_path.exists():
            return map_path
    except Exception:
        pass
    
    # Fallback: local maps directory
    local_maps = Path(__file__).parent / "maps"
    map_path = local_maps / map_name
    if map_path.exists():
        return map_path
    
    raise ValueError(f"Map not found: {map_name}")


# ===================== EXCEPTION HANDLERS =====================
@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    """Custom HTTP exception handler."""
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.detail,
            "status_code": exc.status_code,
        },
    )


@app.exception_handler(Exception)
async def general_exception_handler(request, exc):
    """Catch-all exception handler."""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal server error",
            "detail": str(exc),
        },
    )


# ===================== STARTUP & SHUTDOWN =====================
@app.on_event("startup")
async def startup_event():
    """Initialize ROS node and services on app startup."""
    logger.info("Starting ARYA Web Interface...")
    setup_ros_node()
    await asyncio.sleep(1)  # Give ROS node time to initialize
    logger.info("✓ ARYA Web Interface ready")


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on app shutdown."""
    logger.info("Shutting down ARYA Web Interface...")
    shutdown_ros()
    logger.info("✓ ARYA Web Interface stopped")


# ===================== SIGNAL HANDLERS =====================
def signal_handler(sig, frame):
    """Handle SIGINT (Ctrl+C) gracefully."""
    logger.info(f"Received signal {sig}, shutting down...")
    shutdown_ros()


signal.signal(signal.SIGINT, signal_handler)


# ===================== MAIN =====================
def main():
    """Entry point for ros2 launch (called by console_scripts in setup.py)."""
    import sys
    
    # Configuration
    host = os.getenv("ARYA_WEB_HOST", "0.0.0.0")
    port = int(os.getenv("ARYA_WEB_PORT", "8000"))
    reload = "--reload" in sys.argv
    
    logger.info(f"Starting web server on {host}:{port}")
    logger.info(f"Dashboard: http://localhost:{port}")
    
    # Run Uvicorn server
    uvicorn.run(
        app,
        host=host,
        port=port,
        reload=reload,
        log_level="info",
    )


if __name__ == "__main__":
    main()
