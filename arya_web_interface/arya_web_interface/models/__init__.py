"""
Pydantic models for HTTP request/response validation.
Ensures type safety and input validation across the API.
"""

from .requests import (
    LocalizationRequest,
    MappingSaveRequest,
    NavAnnotationsRequest,
)

__all__ = [
    "LocalizationRequest",
    "MappingSaveRequest",
    "NavAnnotationsRequest",
]
