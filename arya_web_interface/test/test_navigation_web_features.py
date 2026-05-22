from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WEB_NODE = PACKAGE_ROOT / "arya_web_interface" / "web_node.py"
INDEX_HTML = PACKAGE_ROOT / "arya_web_interface" / "static" / "index.html"
SRC_ROOT = PACKAGE_ROOT.parent
HEADLESS_NAV_PARAMS = SRC_ROOT / "amr_bringup_headless" / "config" / "nav2_params.yaml"
HEADLESS_NAV_LAUNCH = SRC_ROOT / "amr_bringup_headless" / "launch" / "navigation.launch.py"


def read_text(path):
    return path.read_text(encoding="utf-8")


def test_backend_streams_scan_to_navigation_websocket():
    source = read_text(WEB_NODE)

    assert "from sensor_msgs.msg import LaserScan" in source
    assert "self.sub_lidar_scan = self.create_subscription(" in source
    assert "LaserScan, '/scan', self.cb_lidar_scan" in source
    assert "def cb_lidar_scan(self, msg: LaserScan):" in source
    assert '"type": "nav_lidar_scan"' in source


def test_backend_transforms_grids_and_sends_nav2_action_goals():
    source = read_text(WEB_NODE)

    assert "from nav2_msgs.action import NavigateToPose" in source
    assert "from rclpy.action import ActionClient" in source
    assert "ActionClient(self, NavigateToPose, 'navigate_to_pose')" in source
    assert 'lookup_transform("map", source_frame, Time())' in source
    assert '"transform_ok": transformed["transform_ok"]' in source
    assert "self.path_data = []" in source
    assert "wait_for_server(timeout_sec=0.05)" in source
    assert '"type"] = "goal_pose_ack"' in source
    assert '"nav_goal_status": ROS_NODE.get_nav_goal_status()' in source


def test_navigation_canvas_draws_amcl_aligned_costmaps_lidar_and_drag_pose_tools():
    html = read_text(INDEX_HTML)

    assert 'id="navShowLidar" checked' in html
    assert "window.requestNavDraw" in html
    assert "window.onNavLidarScan" in html
    assert "drawLidarScan(lidarScan)" in html
    assert "drawGrid(localBitmap, localCostmap)" in html
    assert "nctx.rotate(-(Number(data.oyaw) || 0))" in html
    assert "navPose.x - mw / 2" not in html
    assert "navPose.y - mh / 2" not in html
    assert "navCanvas.addEventListener('pointerdown'" in html
    assert "POSE_DRAG_THRESHOLD_PX" in html
    assert "drawPoseMarker(poseDraft)" in html
    assert "safeSend({ type: 'goal_pose', x, y, theta })" in html


def test_drive_canvas_has_adaptive_meter_grid_and_nav_style_pan_zoom():
    html = read_text(INDEX_HTML)

    assert 'id="p-canvas"' in html
    assert 'id="odomCanvas"' in html
    assert 'id="odomGridScale"' in html
    assert "function odomGridStepMeters()" in html
    assert "const candidates = [1, 5, 10]" in html
    assert "screenToOdomWorld" in html
    assert "odomWorldToScreen" in html
    assert "drawTrajectoryPath()" in html
    assert "canvas.addEventListener('pointerdown'" in html
    assert "canvas.addEventListener('wheel'" in html
    assert "zoomOdomAtPoint" in html


def test_navigation_restrictions_stations_and_queue_controls_are_wired():
    source = read_text(WEB_NODE)
    html = read_text(INDEX_HTML)

    assert "@app.get(\"/api/nav_annotations\")" in source
    assert "@app.post(\"/api/nav_annotations\")" in source
    assert "selected_keepout_mask.txt" in source
    assert "write_keepout_mask(map_path, clean_zones)" in source
    assert "def start_station_queue(self, stations):" in source
    assert "mission_queue_start" in source
    assert "mission_queue_cancel" in source

    assert 'id="p-nav-log"' in html
    assert 'data-panel="p-nav-log" data-view="navigation"' in html
    assert 'id="navLogStream"' in html
    assert "addNavLog('Mission'" in html
    assert "addNavLog('Restriction'" in html
    assert 'id="p-local"' not in html
    assert 'data-panel="p-local" data-view="navigation"' not in html
    assert 'data-panel="p-local" data-view="drive"' not in html
    assert 'id="p-nav-zones"' in html
    assert 'id="btnDrawKeepout"' in html
    assert 'id="btnStartQueue"' in html
    assert "safeSend({ type: 'mission_queue_start'" in html
    assert "fetch('/api/nav_annotations'" in html
    assert "amr_bringup_headless/maps" in html


def test_navigation_map_picker_is_header_modal_with_preview_and_no_local_panel_dependency():
    html = read_text(INDEX_HTML)

    assert 'id="btnChooseMap"' in html
    assert 'id="selectedMapName"' in html
    assert 'id="modalChooseMap"' in html
    assert 'role="dialog" aria-modal="true" aria-labelledby="chooseMapTitle"' in html
    assert 'for="modalMapSelect"' in html
    assert 'id="modalMapSelect"' in html
    assert 'id="mapPreviewCanvas"' in html
    assert 'role="img"' in html
    assert 'id="chooseMapStatus" role="status" aria-live="polite"' in html
    assert 'id="btnConfirmChooseMap"' in html
    assert "function openMapChooser()" in html
    assert "function drawMapPreview(mapName)" in html
    assert "function setSelectedMapUi(mapName)" in html
    assert "window.selectedMapName" in html
    assert "document.dispatchEvent(new CustomEvent('arya:map-selected'" in html
    assert "const mapSelect = document.getElementById('mapSelect')" not in html
    assert "localizeStatus" not in html
    assert "localizeHint" not in html


def test_headless_nav2_keepout_filter_is_configured():
    params = read_text(HEADLESS_NAV_PARAMS)
    launch = read_text(HEADLESS_NAV_LAUNCH)

    assert 'filters: ["keepout_filter"]' in params
    assert 'plugin: "nav2_costmap_2d::KeepoutFilter"' in params
    assert "keepout_filter_mask_server:" in params
    assert "keepout_costmap_filter_info_server:" in params
    assert "selected_keepout_mask.txt" in launch
    assert "executable='costmap_filter_info_server'" in launch
    assert "name='lifecycle_manager_keepout_zone'" in launch
