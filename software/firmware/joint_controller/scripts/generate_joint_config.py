#!/usr/bin/env python3
"""
Post-build script to automatically regenerate joint_config.json.

This script runs after successful firmware build and ensures that the
host application's joint configuration stays synchronized with the
firmware's config_presets.h.

Invoked by PlatformIO via extra_scripts in platformio.ini.
"""

Import("env")
import subprocess
import sys
from pathlib import Path


def generate_joint_config(source, target, env):
    """
    Generate joint_config.json from config_presets.h.
    
    This is called after successful firmware build to ensure
    host configuration stays in sync with firmware.
    """
    print("=" * 70)
    print("POST-BUILD: Generating joint_config.json...")
    print("=" * 70)
    
    # Paths relative to firmware directory
    firmware_dir = Path(env["PROJECT_DIR"])
    software_dir = firmware_dir.parent.parent
    extract_script = software_dir / "scripts" / "extract_joint_config.py"
    config_header = firmware_dir / "include" / "config_presets.h"
    output_json = software_dir / "joint_config.json"
    
    # Validate paths
    if not extract_script.exists():
        print(f"⚠️  Warning: Extract script not found: {extract_script}")
        print("   Skipping joint config generation")
        return
    
    if not config_header.exists():
        print(f"⚠️  Warning: Config header not found: {config_header}")
        print("   Skipping joint config generation")
        return
    
    # Run extraction script
    try:
        result = subprocess.run(
            [sys.executable, str(extract_script), str(config_header), str(output_json)],
            capture_output=True,
            text=True,
            check=True
        )
        
        # Print script output
        if result.stdout:
            print(result.stdout)
        
        print("✅ Joint config generation successful!")
        print(f"   Generated: {output_json}")
        print("=" * 70)
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Error generating joint config:")
        print(f"   {e}")
        if e.stdout:
            print(f"   stdout: {e.stdout}")
        if e.stderr:
            print(f"   stderr: {e.stderr}")
        print("=" * 70)
        # Don't fail the build, just warn
        print("⚠️  Build continues, but joint config may be out of sync")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        print("=" * 70)
        print("⚠️  Build continues, but joint config may be out of sync")


# Register post-build action
# This runs after the firmware .elf is built successfully
env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", generate_joint_config)

print("📝 Registered post-build action: generate_joint_config")

