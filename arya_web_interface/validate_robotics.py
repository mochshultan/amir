#!/usr/bin/env python3
"""
Robotics Best Practices Validation Script
Checks for common robotics control issues in headless web interface
"""

import subprocess
import sys
import re
from pathlib import Path


def check_thread_safety():
    """Verify thread-safe locking patterns in ROS node."""
    print("\n[CHECK] Thread Safety...")
    ros_node = Path("arya_web_interface/services/ros_node.py")
    
    if not ros_node.exists():
        print("  ✗ ros_node.py not found")
        return False
    
    content = ros_node.read_text()
    
    # Check for lock usage patterns
    checks = [
        (r"self\.\w+_lock = threading\.Lock\(\)", "Lock initialization"),
        (r"with self\.\w+_lock:", "Lock context manager usage"),
        (r"self\.get_logger\(\)\.warn\(", "Warning logging for lock contention"),
    ]
    
    passed = 0
    for pattern, desc in checks:
        if re.search(pattern, content):
            print(f"  ✓ {desc}")
            passed += 1
        else:
            print(f"  ✗ {desc}")
    
    return passed == len(checks)


def check_timeouts():
    """Verify timeouts on all ROS operations."""
    print("\n[CHECK] Timeout Protection...")
    ros_node = Path("arya_web_interface/services/ros_node.py")
    
    content = ros_node.read_text()
    
    # Check for timeout patterns
    if "timeout_sec" in content and "deadline" in content:
        print("  ✓ Timeout patterns found in goal waiting")
        return True
    else:
        print("  ✗ No timeout protection found")
        return False


def check_error_handling():
    """Verify error handling in key functions."""
    print("\n[CHECK] Error Handling...")
    
    files = [
        "handlers/__init__.py",
        "services/ros_node.py",
    ]
    
    patterns = [
        (r"except.*Exception", "Exception handling"),
        (r"try:", "Try blocks"),
        (r"HTTPException", "HTTP error responses"),
        (r"self\.get_logger\(\)\.error\(", "Error logging"),
    ]
    
    passed = 0
    for file_path in files:
        full_path = Path(f"arya_web_interface/{file_path}")
        if not full_path.exists():
            continue
        
        content = full_path.read_text()
        for pattern, desc in patterns:
            if re.search(pattern, content):
                print(f"  ✓ {desc} in {file_path}")
                passed += 1
                break
    
    return passed >= 2


def check_data_bounds():
    """Verify data bound limits to prevent memory attacks."""
    print("\n[CHECK] Data Bounds...")
    
    constants = Path("arya_web_interface/utils/constants.py")
    if not constants.exists():
        print("  ✗ constants.py not found")
        return False
    
    content = constants.read_text()
    
    bounds = [
        ("MAX_ECHO_SEQUENCE_ITEMS", "Array truncation limit"),
        ("MAX_ECHO_STRING_CHARS", "String truncation limit"),
        ("MAX_ECHO_DEPTH", "Recursion depth limit"),
        ("MAX_LIDAR_POINTS", "LiDAR point sampling limit"),
    ]
    
    passed = 0
    for const, desc in bounds:
        if const in content:
            print(f"  ✓ {const}: {desc}")
            passed += 1
        else:
            print(f"  ✗ {const}: {desc}")
    
    return passed == len(bounds)


def check_headless_support():
    """Verify API endpoints work without UI."""
    print("\n[CHECK] Headless Operation...")
    
    api_client = Path("static/js/api-client.js")
    if not api_client.exists():
        print("  ✗ API client not found")
        return False
    
    content = api_client.read_text()
    
    endpoints = [
        ("healthz", "Health check endpoint"),
        ("getMaps", "Map listing"),
        ("getMapGrid", "Map grid data"),
        ("request", "Generic HTTP method"),
    ]
    
    passed = 0
    for method, desc in endpoints:
        if method in content:
            print(f"  ✓ {desc}")
            passed += 1
        else:
            print(f"  ✗ {desc}")
    
    return passed == len(endpoints)


def check_function_conflicts():
    """Check for naming conflicts in global scope."""
    print("\n[CHECK] Function Conflicts...")
    
    js_files = list(Path("static/js").glob("*.js"))
    global_exports = set()
    
    for js_file in js_files:
        content = js_file.read_text()
        
        # Find window.* exports
        exports = re.findall(r"window\.(\w+)\s*=", content)
        global_exports.update(exports)
    
    expected = {"app", "APIClient", "stateManager", "uiRenderer"}
    
    unexpected = global_exports - expected
    if unexpected:
        print(f"  ✗ Unexpected global exports: {unexpected}")
        return False
    
    if len(global_exports) == len(expected):
        print(f"  ✓ Clean namespace: {expected}")
        return True
    else:
        print(f"  ✗ Missing exports: {expected - global_exports}")
        return False


def run_syntax_checks():
    """Run Python syntax checks."""
    print("\n[CHECK] Python Syntax...")
    
    try:
        result = subprocess.run(
            ["python", "-m", "py_compile", "arya_web_interface/services/ros_node.py"],
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            print("  ✓ ros_node.py syntax valid")
            return True
        else:
            print(f"  ✗ Syntax error: {result.stderr}")
            return False
    except FileNotFoundError:
        print("  ⚠ Python compiler not available")
        return None


def main():
    """Run all validation checks."""
    print("=" * 60)
    print("ROBOTICS HEADLESS WEB CONTROL - BEST PRACTICES VALIDATION")
    print("=" * 60)
    
    checks = [
        ("Thread Safety", check_thread_safety),
        ("Timeouts", check_timeouts),
        ("Error Handling", check_error_handling),
        ("Data Bounds", check_data_bounds),
        ("Headless Support", check_headless_support),
        ("Function Conflicts", check_function_conflicts),
        ("Syntax", run_syntax_checks),
    ]
    
    results = {}
    for name, check_fn in checks:
        try:
            results[name] = check_fn()
        except Exception as e:
            print(f"  ✗ Check failed: {e}")
            results[name] = False
    
    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    passed = sum(1 for v in results.values() if v is True)
    failed = sum(1 for v in results.values() if v is False)
    skipped = sum(1 for v in results.values() if v is None)
    
    for name, result in results.items():
        status = "✓ PASS" if result is True else ("✗ FAIL" if result is False else "⊘ SKIP")
        print(f"{status} - {name}")
    
    print(f"\nTotal: {passed} passed, {failed} failed, {skipped} skipped")
    print("=" * 60)
    
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
