"""Map loading and grid data utilities."""

import base64
import ast
import math
from pathlib import Path


def read_pgm_token(stream) -> str:
    """
    Read a whitespace/comment-delimited token from PGM file stream.
    
    Args:
        stream: Binary file stream
        
    Returns:
        Decoded ASCII token
    """
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
    """
    Load PGM image file (P5 binary or P2 ASCII format).
    
    Converts to 8-bit grayscale if needed.
    
    Args:
        image_path: Path to .pgm file
        
    Returns:
        Tuple of (width, height, pixel_bytes)
        
    Raises:
        ValueError: If format is invalid or data is incomplete
    """
    with image_path.open("rb") as stream:
        magic = read_pgm_token(stream)
        if magic not in ("P5", "P2"):
            raise ValueError(f"Format image {image_path.name} bukan PGM P5/P2.")
        width = int(read_pgm_token(stream))
        height = int(read_pgm_token(stream))
        max_value = int(read_pgm_token(stream))
        if width <= 0 or height <= 0 or max_value <= 0 or max_value > 65535:
            raise ValueError(f"Header PGM {image_path.name} tidak didukung.")

        if magic == "P5":
            pixels = stream.read(width * height)
            if len(pixels) != width * height:
                raise ValueError(f"Data PGM {image_path.name} tidak lengkap.")
            if max_value != 255:
                pixels = bytes(int(round(pixel * 255.0 / max_value)) for pixel in pixels)
            return width, height, pixels

        # ASCII format P2
        values = []
        while len(values) < width * height:
            token = read_pgm_token(stream)
            if not token:
                break
            values.append(int(token))
        if len(values) != width * height:
            raise ValueError(f"Data PGM ASCII {image_path.name} tidak lengkap.")
        return width, height, bytes(
            max(0, min(255, int(round(value * 255.0 / max_value))))
            for value in values
        )


def parse_map_yaml(map_path: Path) -> dict:
    """
    Parse map metadata from YAML file.
    
    Args:
        map_path: Path to .yaml map config
        
    Returns:
        Dictionary with map metadata (image path, resolution, origin, etc)
        
    Raises:
        ValueError: If YAML format is invalid or missing required fields
    """
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


def map_pixels_to_occupancy(metadata: dict, width: int, height: int, pixels: bytes) -> bytes:
    """
    Convert image pixels to ROS occupancy grid format.
    
    Uses occupancy thresholds to classify pixels as free (0), occupied (100),
    or unknown (-1) depending on configured mode.
    
    Args:
        metadata: Map metadata dict from parse_map_yaml()
        width: Image width in pixels
        height: Image height in pixels
        pixels: Raw pixel bytes (0-255)
        
    Returns:
        Occupancy grid bytes (0-100 or -1 for unknown)
    """
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
    """
    Load complete map grid data for frontend visualization.
    
    Args:
        map_path: Path to .yaml map config
        
    Returns:
        Dictionary with width, height, resolution, origin, and base64-encoded grid
        
    Raises:
        ValueError, FileNotFoundError, OSError: Various file/format errors
    """
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


def point_in_polygon(x: float, y: float, points: list[list[float]]) -> bool:
    """
    Ray-casting algorithm for point-in-polygon test.
    
    Args:
        x, y: Point coordinates
        points: List of [x, y] polygon vertices
        
    Returns:
        True if point is inside polygon
    """
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
