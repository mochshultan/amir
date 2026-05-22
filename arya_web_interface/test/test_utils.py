"""
Unit tests for backend utilities and validation
Run with: pytest test/test_utils.py -v
"""

import pytest
import math
from pathlib import Path
from arya_web_interface.utils.validation import (
    validate_coordinates,
    validate_map_name,
    sanitize_mapping_file_stem,
)
from arya_web_interface.utils.converters import (
    normalize_topic_name,
    ros_value_to_bounded_data,
)


class TestValidation:
    """Test input validation functions."""

    def test_validate_coordinates_valid(self):
        """Valid coordinates should pass."""
        x, y, theta = validate_coordinates(1.5, 2.3, 0.785)
        assert x == 1.5
        assert y == 2.3
        assert theta == 0.785

    def test_validate_coordinates_default_theta(self):
        """Default theta should be 0."""
        x, y, theta = validate_coordinates(1.0, 2.0)
        assert theta == 0.0

    def test_validate_coordinates_invalid_nan(self):
        """NaN should raise ValueError."""
        with pytest.raises(ValueError):
            validate_coordinates(float("nan"), 0, 0)

    def test_validate_coordinates_invalid_inf(self):
        """Infinity should raise ValueError."""
        with pytest.raises(ValueError):
            validate_coordinates(float("inf"), 0, 0)

    def test_validate_coordinates_string_convert(self):
        """String numbers should convert."""
        x, y, theta = validate_coordinates("1.5", "2.3", "0.5")
        assert isinstance(x, float)

    def test_validate_map_name_valid(self):
        """Valid map names should pass."""
        assert validate_map_name("test_map.yaml") == "test_map.yaml"
        assert validate_map_name("map_20240101.yaml") == "map_20240101.yaml"

    def test_validate_map_name_path_traversal(self):
        """Path traversal attempts should fail."""
        with pytest.raises(ValueError):
            validate_map_name("../../../etc/passwd.yaml")
        
        with pytest.raises(ValueError):
            validate_map_name("map/subdir/test.yaml")

    def test_validate_map_name_no_extension(self):
        """Map names must end with .yaml."""
        with pytest.raises(ValueError):
            validate_map_name("mapname")

    def test_sanitize_mapping_file_stem_valid(self):
        """Valid stems should pass."""
        assert sanitize_mapping_file_stem("my_map") == "my_map"
        assert sanitize_mapping_file_stem("map20240101") == "map20240101"

    def test_sanitize_mapping_file_stem_remove_extension(self):
        """Extensions should be stripped."""
        assert sanitize_mapping_file_stem("map.yaml") == "map"
        assert sanitize_mapping_file_stem("map.pgm") == "map"

    def test_sanitize_mapping_file_stem_invalid_chars(self):
        """Invalid characters should fail."""
        with pytest.raises(ValueError):
            sanitize_mapping_file_stem("map@!$%")


class TestConverters:
    """Test data conversion functions."""

    def test_normalize_topic_name_empty(self):
        """Empty string should return empty."""
        assert normalize_topic_name("") == ""
        assert normalize_topic_name(None) == ""

    def test_normalize_topic_name_add_slash(self):
        """Should add leading slash if missing."""
        assert normalize_topic_name("cmd_vel") == "/cmd_vel"

    def test_normalize_topic_name_keep_slash(self):
        """Should preserve leading slash."""
        assert normalize_topic_name("/cmd_vel") == "/cmd_vel"

    def test_ros_value_to_bounded_data_primitives(self):
        """Primitives should pass through."""
        assert ros_value_to_bounded_data(42) == 42
        assert ros_value_to_bounded_data(3.14) == 3.14
        assert ros_value_to_bounded_data(True) is True
        assert ros_value_to_bounded_data(None) is None

    def test_ros_value_to_bounded_data_string_truncate(self):
        """Long strings should be truncated."""
        long_str = "x" * 2000
        result = ros_value_to_bounded_data(long_str)
        assert len(result) < len(long_str)
        assert "truncated" in result

    def test_ros_value_to_bounded_data_list_limit(self):
        """Large lists should be limited."""
        large_list = list(range(100))
        result = ros_value_to_bounded_data(large_list)
        assert isinstance(result, dict)
        assert result.get("truncated") is True
        assert "sample" in result

    def test_ros_value_to_bounded_data_depth_limit(self):
        """Deep nesting should stop at limit."""
        deep_dict = {"a": {"b": {"c": {"d": {"e": {"f": "value"}}}}}}
        result = ros_value_to_bounded_data(deep_dict)
        # Should have stopped at depth limit and returned string repr
        assert isinstance(result, dict)


class TestThreadSafety:
    """Test thread safety of key functions."""

    def test_concurrent_validation_calls(self):
        """Validation should work concurrently."""
        import threading
        
        results = []
        errors = []
        
        def validate_worker(value):
            try:
                result = validate_coordinates(value[0], value[1], value[2])
                results.append(result)
            except Exception as e:
                errors.append(e)
        
        threads = [
            threading.Thread(target=validate_worker, args=((1.0 + i*0.1, 2.0 + i*0.2, 0.5),))
            for i in range(10)
        ]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        assert len(results) == 10
        assert len(errors) == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
