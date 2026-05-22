"""HTTP request handlers for web API endpoints."""

import json
import subprocess
import asyncio
import re
import time
import base64
from pathlib import Path
from fastapi import HTTPException

from ..utils.validation import (
    validate_map_name,
    sanitize_mapping_file_stem,
)
from ..utils.maps import (
    load_static_map_grid,
    parse_map_yaml,
    read_pgm_image,
    map_pixels_to_occupancy,
    point_in_polygon,
    write_keepout_mask,
)


async def handle_get_maps(find_maps_fn, extract_default_fn) -> dict:
    """
    Get list of available maps and selected map.
    
    Args:
        find_maps_fn: Function to find maps directory
        extract_default_fn: Function to extract default map name
        
    Returns:
        Dict with available maps and selected map
    """
    maps_dir = find_maps_fn()
    if maps_dir is None:
        return {"maps": [], "selected_map": None}
    
    available_maps = sorted(
        [
            path.name
            for path in maps_dir.glob("*.yaml")
            if path.is_file() and not path.name.endswith("_keepout.yaml")
        ],
        key=str.casefold,
    )
    selected_map = extract_default_fn(maps_dir)
    
    if selected_map not in available_maps:
        selected_map = available_maps[0] if available_maps else None
    
    return {
        "maps": available_maps,
        "selected_map": selected_map,
    }


async def handle_get_map_grid(map_name: str, resolve_map_fn) -> dict:
    """
    Get OccupancyGrid data for map visualization.
    
    Args:
        map_name: Map YAML filename
        resolve_map_fn: Function to resolve map path safely
        
    Returns:
        Grid dictionary with base64-encoded data
        
    Raises:
        HTTPException: On validation or file errors
    """
    try:
        map_path = resolve_map_fn(map_name)
        return await asyncio.to_thread(load_static_map_grid, map_path)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"Gagal membaca map: {exc}") from exc


async def handle_get_nav_annotations(
    map_name: str,
    resolve_map_fn,
    nav_annotation_path_fn,
) -> dict:
    """
    Get navigation annotations (keepout zones, stations) for a map.
    
    Args:
        map_name: Map YAML filename
        resolve_map_fn: Function to resolve map path safely
        nav_annotation_path_fn: Function to get annotation file path
        
    Returns:
        Dict with zones, stations, and file paths
        
    Raises:
        HTTPException: On validation or file errors
    """
    try:
        map_path = resolve_map_fn(map_name)
        annotations_path = nav_annotation_path_fn(map_path)
        
        if not annotations_path.exists():
            return {
                "map_name": map_path.name,
                "zones": [],
                "stations": [],
                "annotation_path": str(annotations_path),
                "keepout_yaml": "",
                "keepout_pgm": "",
            }
        
        data = json.loads(annotations_path.read_text(encoding="utf-8"))
        return {
            "map_name": map_path.name,
            "zones": data.get("zones", []),
            "stations": data.get("stations", []),
            "annotation_path": str(annotations_path),
            "keepout_yaml": "",
            "keepout_pgm": "",
        }
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except (json.JSONDecodeError, OSError) as exc:
        raise HTTPException(status_code=500, detail=f"Gagal membaca annotation: {exc}") from exc


async def handle_save_slam_map(
    raw_name: str,
    find_maps_fn,
    ros_logger,
) -> dict:
    """
    Save SLAM mapping output (map.pgm + map.yaml) via ROS service.
    
    Args:
        raw_name: Unsanitized map name stem
        find_maps_fn: Function to find maps directory
        ros_logger: ROS logger for info/error messages
        
    Returns:
        Dict with save success status and file paths
        
    Raises:
        HTTPException: On validation, ROS service, or file errors
    """
    try:
        stem = sanitize_mapping_file_stem(raw_name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    
    maps_dir = find_maps_fn()
    if maps_dir is None:
        raise HTTPException(status_code=500, detail="Maps directory not found")
    
    file_prefix = str((maps_dir / stem).resolve())
    
    save_args = [
        "ros2", "service", "call",
        "/slam_toolbox/save_map",
        "slam_toolbox/srv/SaveMap",
        f"{{name: {{data: '{file_prefix}'}}}}",
    ]
    
    try:
        result = subprocess.run(
            ["bash", "-ic", " ".join(save_args)],
            capture_output=True,
            text=True,
            timeout=30.0,
            cwd=str(maps_dir),
        )
        
        if result.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail=f"SLAM save failed: {result.stderr or result.stdout}"
            )
    except subprocess.TimeoutExpired as exc:
        raise HTTPException(status_code=500, detail=f"SLAM save timeout: {exc}") from exc
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"Failed to run SLAM save: {exc}") from exc
    
    yaml_path = maps_dir / f"{stem}.yaml"
    pgm_path = maps_dir / f"{stem}.pgm"
    
    if not yaml_path.exists() or not pgm_path.exists():
        raise HTTPException(
            status_code=500,
            detail=f"SLAM save completed but files not found"
        )
    
    ros_logger.info(f"Map saved: {stem}.yaml, {stem}.pgm")
    
    return {
        "ok": True,
        "message": f"Map {stem} saved successfully",
        "map_name": f"{stem}.yaml",
        "yaml_path": str(yaml_path),
        "pgm_path": str(pgm_path),
    }
