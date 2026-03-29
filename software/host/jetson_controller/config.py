"""
Configuration loader for the Jetson controller.

Merges two sources:
  1. joint_config.json  (auto-generated from firmware) — topology, limits
  2. controller.yaml    (our file) — gains, stiffness, CAN settings

Validates that every joint in the YAML exists in the JSON and that
DOF indices match.
"""
from __future__ import annotations

import json
import logging
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import serial.tools.list_ports
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
    nudge_speed_deg_s: float
    joints: dict[str, JointControlConfig]  # keyed by UPPER name (e.g. "KNEE_RIGHT")

    # Reverse lookup: joint_id → joint key
    _id_to_key: dict[int, str] = field(default_factory=dict, repr=False)

    def __post_init__(self):
        self._id_to_key = {j.joint_id: k for k, j in self.joints.items()}

    def joint_by_id(self, joint_id: int) -> Optional[JointControlConfig]:
        key = self._id_to_key.get(joint_id)
        return self.joints.get(key) if key else None


def _autodetect_can_channel() -> str:
    """Auto-detect CANable USB adapter by VID:PID.

    Known adapters:
      CANable 2:  VID=16D0  PID=117E

    Falls back to description matching ('canable', 'can') if VID:PID
    is unknown.  Returns the device path, or raises FileNotFoundError.
    """
    # Known CAN adapter VID:PID pairs
    CAN_VID_PIDS = {
        (0x16D0, 0x117E),  # CANable 2
    }
    CAN_DESC_HINTS = ("canable", "canbus", "can adapter")

    ports = serial.tools.list_ports.comports()
    candidates: list[str] = []

    for info in sorted(ports, key=lambda p: p.device):
        vid_pid = (info.vid, info.pid) if info.vid and info.pid else None

        # Match by VID:PID
        if vid_pid in CAN_VID_PIDS:
            candidates.append(info.device)
            continue

        # Match by description
        desc = (info.description or "").lower()
        if any(hint in desc for hint in CAN_DESC_HINTS):
            candidates.append(info.device)

    if not candidates:
        raise FileNotFoundError(
            "No CAN adapter found. Connect a CANable adapter or set "
            "'can.channel' in controller.yaml.\n"
            "  Detected USB devices:\n"
            + "\n".join(
                f"    {p.device}  ({p.description})"
                for p in sorted(ports, key=lambda p: p.device)
            )
        )

    if len(candidates) > 1:
        logger.warning(
            f"Multiple CAN adapters found: {candidates} — using {candidates[0]}"
        )

    logger.info(f"Auto-detected CAN device: {candidates[0]}")
    return candidates[0]


def load_config(yaml_path: Optional[str] = None,
                selected_joints: Optional[list[str]] = None) -> ControllerConfig:
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
    if not yaml_joints:
        raise ValueError("controller.yaml does not define any joint profiles")

    available_joint_keys = {joint_key.lower(): joint_yaml for joint_key, joint_yaml in yaml_joints.items()}
    requested_joint_keys: list[str]

    if selected_joints:
        requested_joint_keys = []
        for raw_name in selected_joints:
            for token in str(raw_name).split(","):
                name = token.strip().lower()
                if name:
                    requested_joint_keys.append(name)
        if not requested_joint_keys:
            raise ValueError("Empty --joint selection")
        missing = sorted(set(requested_joint_keys) - set(available_joint_keys.keys()))
        if missing:
            raise ValueError(
                f"Selected joint profile(s) not found in controller.yaml: {missing}. "
                f"Available: {sorted(available_joint_keys.keys())}"
            )
    else:
        default_joint = cfg.get("default_joint")
        if default_joint:
            default_key = str(default_joint).strip().lower()
            if default_key not in available_joint_keys:
                raise ValueError(
                    f"default_joint '{default_joint}' not found in controller.yaml. "
                    f"Available: {sorted(available_joint_keys.keys())}"
                )
            requested_joint_keys = [default_key]
        elif len(available_joint_keys) == 1:
            requested_joint_keys = list(available_joint_keys.keys())
        else:
            raise ValueError(
                "controller.yaml defines multiple joint profiles but no selection was provided. "
                "Set default_joint in controller.yaml or launch with --joint <name>."
            )

    for lower_key in requested_joint_keys:
        joint_yaml = available_joint_keys[lower_key]
        upper_key = lower_key.upper()

        # Find in JSON
        if lower_key not in joint_json["joints"]:
            raise ValueError(
                f"Joint '{lower_key}' in controller.yaml not found in joint_config.json. "
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
        nudge_speed_deg_s=cfg.get("nudge", {}).get("speed_deg_s", 15.0),
        joints=joints,
    )
