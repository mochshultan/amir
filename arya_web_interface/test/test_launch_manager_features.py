from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WEB_NODE = PACKAGE_ROOT / "arya_web_interface" / "web_node.py"
INDEX_HTML = PACKAGE_ROOT / "arya_web_interface" / "static" / "index.html"
WEB_LAUNCH = PACKAGE_ROOT / "launch" / "web_interface.launch.py"


def read_text(path):
    return path.read_text(encoding="utf-8")


def test_launch_presets_include_hardware_localization_and_nav2():
    source = read_text(WEB_NODE)

    assert '"amir_hdl": {' in source
    assert '"alias": "amir_hdl"' in source
    assert '"local_hdl": {' in source
    assert '"nav_hdl": {' in source
    assert '"mapping": {' in source
    assert '"alias": "mapping"' in source
    assert 'for dependent_name in ("nav_hdl", "local_hdl", "mapping")' in source
    assert 'Hardware belum terdeteksi. Jalankan amir_hdl dulu.' in source
    assert '"/slam_toolbox/save_map"' in source
    assert '"/slam_toolbox/serialize_map"' in source


def test_web_interface_launch_is_web_only():
    source = read_text(WEB_LAUNCH)

    assert "package='arya_web_interface'" in source
    assert "executable='web_node'" in source
    assert "imu_node" not in source
    assert "motor_driver" not in source
    assert "odom_bridge" not in source
    assert "ekf_node" not in source
    assert "IncludeLaunchDescription" not in source


def test_frontend_launch_control_has_hardware_tile_and_wcag_basics():
    html = read_text(INDEX_HTML)

    assert "alias: amir_hdl" in html
    assert 'id="btnLaunchHardware"' in html
    assert 'id="launchStateHardware"' in html
    assert 'aria-label="Start Hardware launch amir_hdl"' in html
    assert 'aria-pressed="false"' in html
    assert "alias: mapping" in html
    assert 'id="btnLaunchMapping"' in html
    assert 'id="launchStateMapping"' in html
    assert 'id="mappingSavePanel" class="mapping-save-panel" hidden aria-hidden="true"' in html
    assert 'id="btnOpenSaveMap"' in html
    assert 'id="modalSaveMap"' in html
    assert 'fetch(\'/api/mapping/save\'' in html
    assert "function setMappingSavePanelVisible(visible)" in html
    assert "if (status.name === 'mapping') setMappingSavePanelVisible(mappingActive);" in html
    assert "const mappingActive = !!status.running || status.status === 'running';" in html
    assert 'id="p-nav-log"' in html
    assert 'id="launchHint" role="status"' in html
    assert 'aria-live="polite"' in html
    assert "Stop Hardware? Nav2, Localization, and Mapping will be stopped first." in html
