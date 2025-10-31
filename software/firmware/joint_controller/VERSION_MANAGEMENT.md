# Firmware Version Management

This document explains how firmware versions are managed and when/how to update them.

## Version Information

The firmware exposes 4 version-related values:

| Field | Description | How to Update |
|-------|-------------|---------------|
| `FW_VERSION` | Firmware semantic version | Create git tag (see below) |
| `PROTO_VERSION` | Serial protocol version | Edit `scripts/generate_version.py` line 54 |
| `BUILD_GIT_SHA` | Git commit hash | Automatic (from git) |
| `BUILD_DATE` | Build timestamp UTC | Automatic (current time) |

These values are:
- Automatically generated in `include/version.h` by `scripts/generate_version.py`
- Sent to host on startup via `EVT:FW:VERSION`, `EVT:PROTO`, `EVT:BUILD`
- Used for debugging and compatibility checking

## Current Versions

**Firmware**: `0.1.0` (from git tag or default)  
**Protocol**: `0.1` (hardcoded in script)

## How to Update Firmware Version

The firmware version is taken from **git tags**. To release a new version:

### 1. Make sure all changes are committed

```bash
git status  # Should show "nothing to commit, working tree clean"
```

If you have uncommitted changes, commit them first:
```bash
git add .
git commit -m "feat: your changes description"
```

### 2. Create and push a git tag

```bash
# For patch version (0.1.0 → 0.1.1)
git tag v0.1.1

# For minor version (0.1.0 → 0.2.0)
git tag v0.2.0

# For major version (0.1.0 → 1.0.0)
git tag v1.0.0

# Push the tag to remote
git push origin v0.1.1  # or whatever version you tagged
```

### 3. Build the firmware

Next time you build, `FW_VERSION` will automatically be `0.1.1` (or whatever you tagged).

```bash
# Click build button in Cursor
# OR: cd software && make fw-build-controller ENV=pico
```

### 4. Verify the version

Check the pre-build output:
```
======================================================================
PRE-BUILD: Generating version.h...
======================================================================
✅ Version info updated:
   Firmware:  0.1.1
   Protocol:  0.1
   Git SHA:   a1b2c3d4
   Build:     2025-10-20T10:30:00Z
   File:      .../include/version.h
======================================================================
```

Or check the startup messages when running the firmware:
```
EVT:FW:VERSION 0.1.1
EVT:PROTO 0.1
EVT:BUILD a1b2c3d4 2025-10-20T10:30:00Z
```

## How to Update Protocol Version

The protocol version should be bumped when you make **incompatible changes** to the serial communication protocol between host and firmware.

### Examples of Protocol Changes

**Requires version bump** (breaking changes):
- ❌ Changing command format (e.g., `MOVE` → `MOVE_MULTI_DOF`)
- ❌ Changing event format (e.g., adding/removing fields)
- ❌ Changing parameter order
- ❌ Removing commands or events

**Does NOT require version bump** (backward compatible):
- ✅ Adding new optional commands
- ✅ Adding new events (if host ignores unknown events)
- ✅ Bug fixes that don't change protocol
- ✅ Internal firmware improvements

### To Update Protocol Version

1. **Edit the script**:
   ```bash
   vim software/firmware/joint_controller/scripts/generate_version.py
   ```

2. **Change line 54**:
   ```python
   # BEFORE
   PROTO_VERSION = "0.1"
   
   # AFTER (for minor change)
   PROTO_VERSION = "0.2"
   
   # AFTER (for major breaking change)
   PROTO_VERSION = "1.0"
   ```

3. **Commit the change**:
   ```bash
   git add software/firmware/joint_controller/scripts/generate_version.py
   git commit -m "feat: bump protocol version to 0.2"
   ```

4. **Build and verify** (same as firmware version above)

5. **Update host compatibility check** (if you have one):
   ```python
   # In host application
   REQUIRED_PROTO_VERSION = "0.2"
   ```

## Semantic Versioning

We follow [Semantic Versioning](https://semver.org/) for both firmware and protocol versions:

**Format**: `MAJOR.MINOR.PATCH`

### Firmware Version

- **MAJOR** (1.0.0 → 2.0.0): Breaking changes, major rewrite, incompatible API changes
- **MINOR** (0.1.0 → 0.2.0): New features, backward compatible
- **PATCH** (0.1.0 → 0.1.1): Bug fixes, minor improvements

### Protocol Version

- **MAJOR** (0.1 → 1.0): Breaking protocol changes, incompatible with old host
- **MINOR** (0.1 → 0.2): Non-breaking protocol additions, backward compatible

Note: Protocol version typically only uses MAJOR.MINOR (no PATCH).

## Version Checking

The host application can check protocol compatibility on startup:

```python
# Example (not implemented yet)
def check_compatibility(firmware_proto, required_proto):
    fw_major, fw_minor = map(int, firmware_proto.split('.'))
    req_major, req_minor = map(int, required_proto.split('.'))
    
    # Major version must match exactly
    if fw_major != req_major:
        raise IncompatibleProtocolError(
            f"Firmware protocol {firmware_proto} incompatible with "
            f"required {required_proto}"
        )
    
    # Minor version can be >= (backward compatible)
    if fw_minor < req_minor:
        warnings.warn(
            f"Firmware protocol {firmware_proto} older than "
            f"required {required_proto}"
        )
```

## Best Practices

### When to Release

1. **After significant changes**: Don't wait too long, release often
2. **Before hardware testing**: Tag a version before testing on hardware
3. **Before sharing with others**: Always use tagged versions for collaboration
4. **Before merging to main**: Tag release candidates (e.g., `v0.2.0-rc1`)

### Commit Message Convention

When tagging a release, the last commit message should describe what changed:

```bash
# Good commit messages before tagging
git commit -m "feat: add 3-DOF hip support"
git commit -m "fix: correct angle limits for ankle joints"
git commit -m "refactor: unify logging system"

# Then tag
git tag v0.2.0
```

### Development Workflow

```
1. Work on feature branch
   └─ Commits: "feat: add X", "fix: Y", etc.

2. Merge to main (or development branch)

3. Test thoroughly

4. Tag release
   └─ git tag v0.2.0

5. Push tag
   └─ git push origin v0.2.0

6. Build and deploy
```

## Troubleshooting

### Q: Version shows "unknown" or "0.1.0"

**A**: You haven't created any git tags yet. Solution:
```bash
git tag v0.1.0  # Create initial tag
```

### Q: Version shows "unknown-dirty"

**A**: You have uncommitted changes. Either:
- Commit your changes: `git commit -am "your changes"`
- Or ignore (development builds often have -dirty suffix)

### Q: How do I list existing tags?

**A**: Use git commands:
```bash
# List all tags
git tag

# List tags with details
git tag -n

# Show latest tag
git describe --tags --abbrev=0
```

### Q: How do I delete a wrong tag?

**A**: Delete locally and remotely:
```bash
# Delete local tag
git tag -d v0.1.0

# Delete remote tag (if already pushed)
git push origin :refs/tags/v0.1.0
```

### Q: When should I use release candidates?

**A**: For pre-release testing before final version:
```bash
git tag v0.2.0-rc1  # Release candidate 1
git tag v0.2.0-rc2  # Release candidate 2
git tag v0.2.0      # Final release
```

## Files

| File | Purpose | Edit? |
|------|---------|-------|
| `scripts/generate_version.py` | Generates version.h | Only for PROTO_VERSION |
| `include/version.h` | Auto-generated version defines | ❌ Never (auto-generated) |
| `VERSION_MANAGEMENT.md` | This documentation | Only to improve docs |

## Related

- Git tagging: https://git-scm.com/book/en/v2/Git-Basics-Tagging
- Semantic Versioning: https://semver.org/
- Conventional Commits: https://www.conventionalcommits.org/

