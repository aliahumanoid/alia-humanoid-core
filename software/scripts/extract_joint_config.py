#!/usr/bin/env python3
"""
Extract joint configuration from firmware config_presets.h and generate JSON.

This script parses the C++ firmware configuration file and extracts:
- Joint IDs and names
- DOF counts and names
- Angle limits (min/max) for each DOF
- Lookup table entries

The generated JSON serves as the single source of truth for both firmware and host,
preventing configuration drift and ensuring consistency.

Usage:
    python extract_joint_config.py [input_file] [output_file]
    
    Default input:  ../firmware/joint_controller/include/config_presets.h
    Default output: joint_config.json
"""

import re
import json
import sys
from pathlib import Path


def parse_config_presets(file_path):
    """
    Parse config_presets.h and extract joint configurations.
    
    Returns:
        dict: Structured joint configuration data
    """
    with open(file_path, 'r') as f:
        content = f.read()
    
    joints = {}
    
    # Extract CONFIG_LOOKUP table to get joint names and IDs
    lookup_pattern = r'\{"([^"]+)",\s*(\d+),\s*&([A-Z_]+)_CONFIG\}'
    lookup_matches = re.findall(lookup_pattern, content)
    
    # Create mapping: config constant name -> (joint_name, joint_id)
    config_to_info = {}
    for joint_name, joint_id, config_const in lookup_matches:
        config_to_info[f"{config_const}_CONFIG"] = (joint_name, int(joint_id))
    
    # Pattern to match entire joint config blocks
    # Matches: const JointConfig NAME_CONFIG = { ... };
    config_pattern = r'const\s+JointConfig\s+([A-Z_]+_CONFIG)\s*=\s*\{(.*?)\};'
    config_matches = re.finditer(config_pattern, content, re.DOTALL)
    
    for match in config_matches:
        config_const = match.group(1)
        config_body = match.group(2)
        
        # Skip if not in lookup table (shouldn't happen, but safety check)
        if config_const not in config_to_info:
            print(f"Warning: {config_const} not found in CONFIG_LOOKUP, skipping")
            continue
        
        joint_name, joint_id = config_to_info[config_const]
        
        # Extract .name field
        name_match = re.search(r'\.name\s*=\s*"([^"]+)"', config_body)
        config_name = name_match.group(1) if name_match else joint_name
        
        # Extract .dof_count
        dof_count_match = re.search(r'\.dof_count\s*=\s*(\d+)', config_body)
        dof_count = int(dof_count_match.group(1)) if dof_count_match else 1
        
        # Extract .motor_count
        motor_count_match = re.search(r'\.motor_count\s*=\s*(\d+)', config_body)
        motor_count = int(motor_count_match.group(1)) if motor_count_match else 2
        
        # Extract DOF configurations
        dofs = []
        
        # Find the .dofs array
        dofs_pattern = r'\.dofs\s*=\s*\{(.*?)\}\s*,\s*\.motors'
        dofs_match = re.search(dofs_pattern, config_body, re.DOTALL)
        
        if dofs_match:
            dofs_content = dofs_match.group(1)
            
            # Split by DOF blocks (each DOF is a {...} structure)
            # This is tricky because of nested braces, so we'll use a simple counter
            dof_blocks = []
            brace_count = 0
            current_block = ""
            
            for char in dofs_content:
                if char == '{':
                    brace_count += 1
                    current_block += char
                elif char == '}':
                    current_block += char
                    brace_count -= 1
                    if brace_count == 0 and current_block.strip():
                        dof_blocks.append(current_block)
                        current_block = ""
                elif brace_count > 0:
                    current_block += char
            
            # Parse each DOF block
            for dof_idx, dof_block in enumerate(dof_blocks):
                # Extract DOF name
                dof_name_match = re.search(r'\.name\s*=\s*"([^"]+)"', dof_block)
                dof_name = dof_name_match.group(1) if dof_name_match else f"dof_{dof_idx}"
                
                # Extract angle limits
                min_angle_match = re.search(r'\.limits\s*=\s*\{[^}]*\.min_angle\s*=\s*([-\d.]+)f?', dof_block)
                max_angle_match = re.search(r'\.limits\s*=\s*\{[^}]*\.max_angle\s*=\s*([-\d.]+)f?', dof_block)
                
                min_angle = float(min_angle_match.group(1)) if min_angle_match else 0.0
                max_angle = float(max_angle_match.group(1)) if max_angle_match else 0.0
                
                # Extract encoder channel
                encoder_ch_match = re.search(r'\.encoder_channel\s*=\s*(\d+)', dof_block)
                encoder_channel = int(encoder_ch_match.group(1)) if encoder_ch_match else dof_idx
                
                # Extract zero angle offset from zero_mapping section
                zero_offset_match = re.search(r'\.zero_angle_offset\s*=\s*([-\d.]+)f?', dof_block)
                zero_angle_offset = float(zero_offset_match.group(1)) if zero_offset_match else 0.0
                
                # Extract auto-mapping parameters
                auto_mapping_min_match = re.search(r'\.auto_mapping_min_angle\s*=\s*([-\d.]+)f?', dof_block)
                auto_mapping_min_angle = float(auto_mapping_min_match.group(1)) if auto_mapping_min_match else min_angle
                
                auto_mapping_max_match = re.search(r'\.auto_mapping_max_angle\s*=\s*([-\d.]+)f?', dof_block)
                auto_mapping_max_angle = float(auto_mapping_max_match.group(1)) if auto_mapping_max_match else max_angle
                
                auto_mapping_step_match = re.search(r'\.auto_mapping_step\s*=\s*([-\d.]+)f?', dof_block)
                auto_mapping_step = float(auto_mapping_step_match.group(1)) if auto_mapping_step_match else 5.0
                
                dofs.append({
                    'index': dof_idx,
                    'name': dof_name,
                    'min_angle': min_angle,
                    'max_angle': max_angle,
                    'encoder_channel': encoder_channel,
                    'zero_angle_offset': zero_angle_offset,
                    'auto_mapping_min_angle': auto_mapping_min_angle,
                    'auto_mapping_max_angle': auto_mapping_max_angle,
                    'auto_mapping_step': auto_mapping_step
                })
        
        # Create joint entry
        joints[joint_name] = {
            'id': joint_id,
            'config_name': config_name,
            'dof_count': dof_count,
            'motor_count': motor_count,
            'dofs': dofs
        }
    
    return {
        'version': '1.0',
        'source': 'config_presets.h',
        'joints': joints
    }


def main():
    """Main entry point."""
    # Default paths
    script_dir = Path(__file__).parent
    default_input = script_dir / '../firmware/joint_controller/include/config_presets.h'
    default_output = script_dir.parent / 'joint_config.json'
    
    # Parse command line arguments
    input_file = Path(sys.argv[1]) if len(sys.argv) > 1 else default_input
    output_file = Path(sys.argv[2]) if len(sys.argv) > 2 else default_output
    
    # Validate input file exists
    if not input_file.exists():
        print(f"Error: Input file not found: {input_file}")
        sys.exit(1)
    
    print(f"Extracting joint configuration from: {input_file}")
    
    # Parse configuration
    try:
        config = parse_config_presets(input_file)
    except Exception as e:
        print(f"Error parsing config file: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    # Write JSON output
    output_file.parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, 'w') as f:
        json.dump(config, f, indent=2)
    
    print(f"✅ Successfully generated: {output_file}")
    print(f"\nExtracted {len(config['joints'])} joints:")
    for joint_name, joint_data in sorted(config['joints'].items(), key=lambda x: x[1]['id']):
        print(f"  [{joint_data['id']}] {joint_name}: {joint_data['dof_count']} DOF, {joint_data['motor_count']} motors")
        for dof in joint_data['dofs']:
            print(f"      DOF {dof['index']}: {dof['name']} (limits: {dof['min_angle']:.1f}° to {dof['max_angle']:.1f}°)")
            print(f"         Auto-mapping: {dof['auto_mapping_min_angle']:.1f}° to {dof['auto_mapping_max_angle']:.1f}° (step {dof['auto_mapping_step']:.1f}°)")


if __name__ == '__main__':
    main()

