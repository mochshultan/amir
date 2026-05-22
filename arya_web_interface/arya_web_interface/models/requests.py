"""HTTP request models with validation."""

from pydantic import BaseModel, Field
from typing import Optional


class LocalizationRequest(BaseModel):
    """Select a localization map."""
    map_name: str = Field(..., min_length=1, max_length=256)


class MappingSaveRequest(BaseModel):
    """Save SLAM mapping result."""
    map_name: str = Field(..., min_length=1, max_length=80)


class NavAnnotationsRequest(BaseModel):
    """Save navigation annotations (zones, stations)."""
    map_name: str = Field(..., min_length=1, max_length=256)
    zones: list[dict] = Field(default_factory=list)
    stations: list[dict] = Field(default_factory=list)
