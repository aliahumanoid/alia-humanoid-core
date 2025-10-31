"""
Utility functions for the joint controller host application.

This module provides helper functions for data serialization
and type conversion for JSON responses.
"""
import numpy as np


def convert_to_serializable(obj):
    """
    Recursively converts NumPy arrays and nested structures to JSON-serializable types.
    
    Handles:
    - numpy.ndarray → list
    - Nested dictionaries and lists (recursive)
    - Other types → passed through unchanged
    
    Args:
        obj: Object to convert (can be ndarray, dict, list, or primitive)
        
    Returns:
        JSON-serializable version of the input object
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {key: convert_to_serializable(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [convert_to_serializable(item) for item in obj]
    else:
        return obj