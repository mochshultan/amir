"""Input validation utilities with WCAG-compliant error messages."""

import re
import math
from pathlib import Path


def validate_coordinates(x: any, y: any, theta: any = None) -> tuple[float, float, float]:
    """
    Validate and convert coordinates to floats.
    
    Args:
        x, y: Cartesian coordinates
        theta: Optional rotation angle in radians
        
    Returns:
        Tuple of (x, y, theta) as floats
        
    Raises:
        ValueError: If values are not numeric or contain infinities/NaNs
    """
    try:
        x = float(x)
        y = float(y)
        theta = float(theta) if theta is not None else 0.0
    except (TypeError, ValueError) as exc:
        raise ValueError("Koordinat harus berupa nilai numerik.") from exc
    
    if not all(math.isfinite(v) for v in (x, y, theta)):
        raise ValueError("Koordinat tidak boleh berupa infinity atau NaN.")
    
    return x, y, theta


def validate_map_name(map_name: str) -> str:
    """
    Validate map filename for path traversal attacks and invalid chars.
    
    Args:
        map_name: Filename to validate
        
    Returns:
        Sanitized map name
        
    Raises:
        ValueError: If name fails validation
    """
    clean_name = str(map_name or "").strip()
    
    if not clean_name or len(clean_name) > 256:
        raise ValueError("Nama map harus 1-256 karakter.")
    
    # Reject path separators
    if "\\" in clean_name or "/" in clean_name or "\x00" in clean_name:
        raise ValueError("Nama map tidak boleh berisi path separators.")
    
    # Ensure ends with .yaml
    if not clean_name.endswith(".yaml"):
        raise ValueError("Nama map harus berakhir dengan .yaml")
    
    # Reject keepout files
    if "_keepout" in clean_name:
        raise ValueError("Tidak boleh memilih keepout file langsung.")
    
    return clean_name


def sanitize_mapping_file_stem(raw_name: str) -> str:
    """
    Sanitize a filename stem for SLAM mapping output.
    
    Removes common file extensions and validates against injection attacks.
    
    Args:
        raw_name: Unsanitized filename stem
        
    Returns:
        Safe filename stem
        
    Raises:
        ValueError: If stem fails validation
    """
    name = str(raw_name or "").strip()
    
    # Remove common suffixes
    for suffix in (".yaml", ".pgm", ".posegraph", ".data"):
        if name.lower().endswith(suffix):
            name = name[: -len(suffix)]
            break
    
    # Reject path components
    if Path(name).name != name or "\\" in name:
        raise ValueError("Nama file tidak boleh berisi path/folder.")
    
    name = name.strip()
    
    # Only allow safe characters
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,79}", name):
        raise ValueError("Nama file hanya boleh huruf, angka, titik, underscore, dan dash.")
    
    if not name:
        raise ValueError("Nama file tidak boleh kosong.")
    
    return name
