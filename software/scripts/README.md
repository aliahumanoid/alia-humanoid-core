# Software Scripts

Utility scripts for the Alia Humanoid software stack.

## Scripts

### `extract_joint_config.py`

**Purpose**: Extract joint configuration from firmware and generate JSON for host application.

**Usage**:
```bash
# Using default paths
python3 extract_joint_config.py

# Custom paths
python3 extract_joint_config.py <input_config.h> <output_config.json>

# Via Makefile (recommended)
cd ..
make joint-config
```

**Input**: `firmware/joint_controller/include/config_presets.h`  
**Output**: `joint_config.json`

**What it does**:
1. Parses C++ firmware configuration using regex
2. Extracts joint IDs, DOF counts, angle limits, motor counts
3. Generates structured JSON with all joint data
4. Validates consistency and reports issues

**When to run**:
- **Automatically**: When building `joint_controller` firmware (post-build hook)
- After modifying `config_presets.h` (if not building firmware)
- When adding/removing joints
- When changing angle limits or DOF counts
- Automatically via `make host-run` (Makefile integration)

**See also**: `docs/JOINT_CONFIG_SYNC.md` for detailed documentation

---

### Firmware Scripts

These scripts are located in `firmware/joint_controller/scripts/` and are automatically invoked by PlatformIO during build:

#### `firmware/joint_controller/scripts/generate_joint_config.py`

**Purpose**: Post-build hook to automatically regenerate `joint_config.json` after firmware build.

**When it runs**: Automatically after successful firmware compilation (any environment: pico2, pico2_debug, rpipico2, rpipico2_debug)

**What it does**:
1. Runs after firmware `.elf` is built
2. Calls `extract_joint_config.py` with correct paths
3. Regenerates `joint_config.json` in `software/` directory
4. Prints status messages during build
5. Non-blocking: warns if generation fails but doesn't fail build

**Integration**: Configured via `extra_scripts = post:scripts/generate_joint_config.py` in `platformio.ini`

**Benefits**:
- ✅ Zero manual intervention: build firmware → JSON auto-updated
- ✅ Impossible to forget: every build ensures sync
- ✅ Fast iteration: change config, rebuild, done

## Directory Structure

```
scripts/
├── README.md                      # This file
└── extract_joint_config.py        # Joint config extractor
```

## Adding New Scripts

When adding utility scripts to this directory:

1. **Make executable**: `chmod +x script_name.py`
2. **Add shebang**: `#!/usr/bin/env python3` (first line)
3. **Document usage**: Update this README
4. **Integrate**: Add to Makefile if appropriate
5. **Test**: Verify script works in clean environment

## Requirements

Python 3.7+ (uses `pathlib`, f-strings)

No external dependencies required for current scripts.

