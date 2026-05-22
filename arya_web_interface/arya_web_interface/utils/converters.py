"""Data conversion and normalization utilities."""

from .constants import (
    MAX_ECHO_SEQUENCE_ITEMS,
    MAX_ECHO_STRING_CHARS,
    MAX_ECHO_DEPTH,
)


def normalize_topic_name(topic_name: str) -> str:
    """
    Normalize ROS topic name to start with /.
    
    Args:
        topic_name: Raw topic name string
        
    Returns:
        Normalized topic name starting with /
        
    Raises:
        ValueError: If normalization produces empty string
    """
    clean_name = str(topic_name or "").strip()
    if not clean_name:
        return ""
    return clean_name if clean_name.startswith("/") else f"/{clean_name}"


def ros_value_to_bounded_data(value, depth: int = 0) -> any:
    """
    Convert ROS message value to bounded JSON-serializable data.
    
    Implements safety limits to prevent large data structures from
    overwhelming the frontend:
    - String truncation at MAX_ECHO_STRING_CHARS
    - Array sampling with truncation indicators
    - Recursion depth limit
    
    Args:
        value: ROS message value to convert
        depth: Current recursion depth (internal use)
        
    Returns:
        JSON-serializable representation with truncation indicators
    """
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
