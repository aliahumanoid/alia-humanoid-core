"""
Configuration loader for the Jetson controller.

Merges two sources:
  1. joint_config.json  (auto-generated from firmware) — topology, limits
  2. controller.yaml    (our file) — gains, stiffness, CAN settings

Validates that every joint in the YAML exists in the JSON and that
DOF indices match.
"""
from __future__ import annotations

import glob
import json
import logging
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml

logger = logging.getLogger(__name__)


@dataclass
class GainSet:
    kp: float
    ki: float
    kd: float


@dataclass
class JointControlConfig:
    """Merged config for a single controlled joint."""
    name: str                              # e.g. "knee_right"
    joint_id: int                          # from JSON
    dof_count: int                         # from JSON
    dof_names: list[str]                   # from JSON
    min_angles: dict[int, float]           # from JSON, per DOF
    max_angles: dict[int, float]           # from JSON, per DOF
    gains_outer: GainSet                   # from YAML
    gains_inner: GainSet                   # from YAML
    stiffness_deg: float                   # from YAML
    home_position_deg: dict[int, float]    # from YAML


@dataclass
class ControllerConfig:
    """Top-level configuration."""
    can_interface: str
    can_channel: str
    can_bitrate: int
    send_rate_hz: int
    watchdog_ms: int
    homing_speed_deg_s: float
    homing_tolerance_deg: float
    joints: dict[str, JointControlConfig]  # keyed by UPPER name (e.g. "KNEE_RIGHT")

    # Reverse lookup: joint_id → joint key
    _id_to_key: dict[int, str] = field(default_factory=dict, repr=False)

    def __post_init__(self):
        self._id_to_key = {j.joint_id: k for k, j in self.joints.items()}

    def joint_by_id(self, joint_id: int) -> Optional[JointControlConfig]:
        key = self._id_to_key.get(joint_id)
        return self.joints.get(key) if key else None


def _autodetect_can_channel() -> str:
    """Auto-detect CANable USB device.

    Search order:
      macOS:  /dev/cu.usbmodem*
      Linux:  /dev/ttyACM*  then  /dev/ttyUSB*

    Returns the device path, or raises FileNotFoundError.
    """
    if sys.platform == "darwin":
        patterns = ["/dev/cu.usbmodem*"]
    else:
        # Linux (Jetson, RPi, etc.)
        patterns = ["/dev/ttyACM*", "/dev/ttyUSB*"]

    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            if len(matches) > 1:
                logger.warning(
                    f"Multiple CAN devices found: {matches} — using {matches[0]}"
                )
            logger.info(f"Auto-detected CAN device: {matches[0]}")
            return matches[0]

    raise FileNotFoundError(
        "No CAN device found. Connect a CANable adapter or set "
        "'can.channel' in controller.yaml.\n"
        f"  Searched: {patterns}"
    )


def load_config(yaml_path: Optional[str] = None) -> ControllerConfig:
    """Load and merge configuration.

    Args:
        yaml_path: Path to controller.yaml. Defaults to config/controller.yaml
                   relative to this file.
    """
    # --- Resolve paths ---
    if yaml_path is None:
        yaml_path = str(Path(__file__).parent / "config" / "controller.yaml")
    yaml_path = Path(yaml_path)

    # joint_config.json lives at software/ level (same as host/config.py expects)
    software_dir = Path(__file__).parent.parent.parent  # host/jetson_controller → host → software
    json_path = software_dir / "joint_config.json"

    # --- Load YAML ---
    if not yaml_path.exists():
        raise FileNotFoundError(f"Controller config not found: {yaml_path}")
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)

    # --- Load JSON (firmware-generated) ---
    if not json_path.exists():
        raise FileNotFoundError(
            f"Joint config not found: {json_path}\n"
            f"Run: python software/scripts/extract_joint_config.py"
        )
    with open(json_path) as f:
        joint_json = json.load(f)

    # --- Merge ---
    joints: dict[str, JointControlConfig] = {}
    yaml_joints = cfg.get("joints", {})

    for joint_key, joint_yaml in yaml_joints.items():
        upper_key = joint_key.upper()
        lower_key = joint_key.lower()

        # Find in JSON
        if lower_key not in joint_json["joints"]:
            raise ValueError(
                f"Joint '{joint_key}' in controller.yaml not found in joint_config.json. "
                f"Available: {list(joint_json['joints'].keys())}"
            )
        jdata = joint_json["joints"][lower_key]

        # Build per-DOF angle limits
        min_angles = {dof["index"]: dof["min_angle"] for dof in jdata["dofs"]}
        max_angles = {dof["index"]: dof["max_angle"] for dof in jdata["dofs"]}
        dof_names = [dof["name"] for dof in jdata["dofs"]]

        # Parse gains from YAML
        outer = joint_yaml["gains"]["outer"]
        inner = joint_yaml["gains"]["inner"]

        # Parse home positions
        home_pos = {}
        for k, v in joint_yaml["home_position_deg"].items():
            dof_idx = int(k)
            if dof_idx >= jdata["dof_count"]:
                raise ValueError(
                    f"Joint '{joint_key}': home_position DOF {dof_idx} exceeds "
                    f"dof_count={jdata['dof_count']}"
                )
            home_pos[dof_idx] = float(v)

        # Validate all DOFs have a home position
        for dof in jdata["dofs"]:
            if dof["index"] not in home_pos:
                raise ValueError(
                    f"Joint '{joint_key}': missing home_position for DOF {dof['index']} "
                    f"({dof['name']})"
                )

        joints[upper_key] = JointControlConfig(
            name=lower_key,
            joint_id=jdata["id"],
            dof_count=jdata["dof_count"],
            dof_names=dof_names,
            min_angles=min_angles,
            max_angles=max_angles,
            gains_outer=GainSet(kp=outer["kp"], ki=outer["ki"], kd=outer["kd"]),
            gains_inner=GainSet(kp=inner["kp"], ki=inner["ki"], kd=inner["kd"]),
            stiffness_deg=float(joint_yaml["stiffness_deg"]),
            home_position_deg=home_pos,
        )

    # --- CAN settings ---
    can_cfg = cfg.get("can", {})
    imp_cfg = cfg.get("impedance", {})
    homing_cfg = cfg.get("homing", {})

    # Auto-detect CAN channel if set to "auto" or empty
    can_channel = can_cfg.get("channel", "auto")
    if not can_channel or can_channel == "auto":
        can_channel = _autodetect_can_channel()

    return ControllerConfig(
        can_interface=can_cfg.get("interface", "slcan"),
        can_channel=can_channel,
        can_bitrate=can_cfg.get("bitrate", 1000000),
        send_rate_hz=imp_cfg.get("send_rate_hz", 50),
        watchdog_ms=imp_cfg.get("watchdog_ms", 2000),
        homing_speed_deg_s=homing_cfg.get("speed_deg_s", 5.0),
        homing_tolerance_deg=homing_cfg.get("tolerance_deg", 1.0),
        joints=joints,
    )
