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

import can
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
    dof_drive_types: dict[int, str]        # from JSON, per DOF
    dof_motor_counts: dict[int, int]       # from JSON, per DOF
    dof_capabilities: dict[int, dict[str, bool]]  # from JSON, per DOF
    min_angles: dict[int, float]           # from JSON, per DOF
    max_angles: dict[int, float]           # from JSON, per DOF
    gains_outer: GainSet                   # default from YAML
    gains_inner: GainSet                   # default from YAML
    stiffness_deg: float                   # default from YAML
    dof_gains_outer: dict[int, GainSet]    # optional per-DOF override from YAML
    dof_gains_inner: dict[int, GainSet]    # optional per-DOF override from YAML
    dof_stiffness_deg: dict[int, float]    # optional per-DOF override from YAML
    home_position_deg: dict[int, float]    # from YAML

    def drive_type_for(self, dof: int) -> str:
        return self.dof_drive_types.get(dof, "antagonistic_tendon")

    def capabilities_for(self, dof: int) -> dict[str, bool]:
        return self.dof_capabilities.get(dof, {})

    def outer_gains_for(self, dof: int) -> GainSet:
        return self.dof_gains_outer.get(dof, self.gains_outer)

    def inner_gains_for(self, dof: int) -> GainSet:
        return self.dof_gains_inner.get(dof, self.gains_inner)

    def stiffness_for(self, dof: int) -> float:
        return self.dof_stiffness_deg.get(dof, self.stiffness_deg)


@dataclass
class ControllerConfig:
    """Top-level configuration."""
    can_interface: str
    can_channel: str
    can_bitrate: int
    startup_pretension_all: bool
    startup_recovery_settle_s: float
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


def _autodetect_can_channel(interface: str = "slcan") -> str:
    """Auto-detect the CAN channel for the selected interface.

    For `slcan`, auto-detects a serial CANable USB adapter by VID:PID.
    For `gs_usb`, defaults to device index ``0`` for the common single-adapter case.
    For `candle`, returns the first detected python-can-candle channel.

    Falls back to description matching ('canable', 'can') if VID:PID
    is unknown.  Returns the device path, or raises FileNotFoundError.
    """
    if interface == "gs_usb":
        logger.info("Auto-selected gs_usb device index 0")
        return "0"

    if interface == "candle":
        configs = can.detect_available_configs("candle")
        if not configs:
            raise FileNotFoundError(
                "No candle backend device found. Ensure python-can-candle is installed "
                "and the CANable is running candleLight firmware."
            )
        channel = str(configs[0].get("channel", 0))
        logger.info("Auto-selected candle device channel %s", channel)
        return channel

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
                selected_joints: Optional[list[str]] = None,
                autodetect_can: bool = True) -> ControllerConfig:
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
        dof_drive_types = {dof["index"]: dof.get("drive_type", "antagonistic_tendon") for dof in jdata["dofs"]}
        dof_motor_counts = {dof["index"]: int(dof.get("motor_count", 0)) for dof in jdata["dofs"]}
        dof_capabilities = {}
        capability_keys = (
            "supports_pretension",
            "supports_recalc_offset",
            "supports_auto_mapping",
            "supports_outer_impedance",
            "supports_slack_diag",
            "supports_retension_probe",
        )
        for dof in jdata["dofs"]:
            idx = dof["index"]
            dof_capabilities[idx] = {
                key: bool(dof.get(key, False)) for key in capability_keys
            }

        # Parse gains from YAML
        outer = joint_yaml["gains"]["outer"]
        inner = joint_yaml["gains"]["inner"]
        dof_overrides = joint_yaml.get("dof_overrides", {})
        if dof_overrides is None:
            dof_overrides = {}
        if not isinstance(dof_overrides, dict):
            raise ValueError(f"Joint '{lower_key}': dof_overrides must be a mapping")

        dof_gains_outer: dict[int, GainSet] = {}
        dof_gains_inner: dict[int, GainSet] = {}
        dof_stiffness_deg: dict[int, float] = {}

        for raw_dof_idx, override_cfg in dof_overrides.items():
            dof_idx = int(raw_dof_idx)
            if dof_idx >= jdata["dof_count"]:
                raise ValueError(
                    f"Joint '{lower_key}': dof_overrides[{dof_idx}] exceeds "
                    f"dof_count={jdata['dof_count']}"
                )
            if not isinstance(override_cfg, dict):
                raise ValueError(
                    f"Joint '{lower_key}': dof_overrides[{dof_idx}] must be a mapping"
                )

            if "stiffness_deg" in override_cfg:
                dof_stiffness_deg[dof_idx] = float(override_cfg["stiffness_deg"])

            gains_cfg = override_cfg.get("gains", {})
            if gains_cfg and not isinstance(gains_cfg, dict):
                raise ValueError(
                    f"Joint '{lower_key}': dof_overrides[{dof_idx}].gains must be a mapping"
                )

            if "outer" in gains_cfg:
                outer_override = gains_cfg["outer"]
                dof_gains_outer[dof_idx] = GainSet(
                    kp=float(outer_override["kp"]),
                    ki=float(outer_override["ki"]),
                    kd=float(outer_override["kd"]),
                )
            if "inner" in gains_cfg:
                inner_override = gains_cfg["inner"]
                dof_gains_inner[dof_idx] = GainSet(
                    kp=float(inner_override["kp"]),
                    ki=float(inner_override["ki"]),
                    kd=float(inner_override["kd"]),
                )

        # Parse home positions
        home_pos = {}
        for k, v in joint_yaml["home_position_deg"].items():
            dof_idx = int(k)
            if dof_idx >= jdata["dof_count"]:
                raise ValueError(
                    f"Joint '{lower_key}': home_position DOF {dof_idx} exceeds "
                    f"dof_count={jdata['dof_count']}"
                )
            home_pos[dof_idx] = float(v)

        # Validate all DOFs have a home position
        for dof in jdata["dofs"]:
            if dof["index"] not in home_pos:
                raise ValueError(
                    f"Joint '{lower_key}': missing home_position for DOF {dof['index']} "
                    f"({dof['name']})"
                )

        joints[upper_key] = JointControlConfig(
            name=lower_key,
            joint_id=jdata["id"],
            dof_count=jdata["dof_count"],
            dof_names=dof_names,
            dof_drive_types=dof_drive_types,
            dof_motor_counts=dof_motor_counts,
            dof_capabilities=dof_capabilities,
            min_angles=min_angles,
            max_angles=max_angles,
            gains_outer=GainSet(kp=outer["kp"], ki=outer["ki"], kd=outer["kd"]),
            gains_inner=GainSet(kp=inner["kp"], ki=inner["ki"], kd=inner["kd"]),
            stiffness_deg=float(joint_yaml["stiffness_deg"]),
            dof_gains_outer=dof_gains_outer,
            dof_gains_inner=dof_gains_inner,
            dof_stiffness_deg=dof_stiffness_deg,
            home_position_deg=home_pos,
        )

    # --- CAN settings ---
    can_cfg = cfg.get("can", {})
    imp_cfg = cfg.get("impedance", {})
    homing_cfg = cfg.get("homing", {})
    can_interface = can_cfg.get("interface", "slcan")

    # Auto-detect CAN channel if set to "auto" or empty
    can_channel = can_cfg.get("channel", "auto")
    if not can_channel or can_channel == "auto":
        if not autodetect_can:
            can_channel = "dry-run"
        else:
            can_channel = _autodetect_can_channel(can_interface)

    return ControllerConfig(
        can_interface=can_interface,
        can_channel=can_channel,
        can_bitrate=can_cfg.get("bitrate", 500000),
        startup_pretension_all=bool(cfg.get("startup", {}).get("pretension_all", False)),
        startup_recovery_settle_s=float(cfg.get("startup", {}).get("recovery_settle_s", 30.0)),
        send_rate_hz=imp_cfg.get("send_rate_hz", 50),
        watchdog_ms=imp_cfg.get("watchdog_ms", 2000),
        homing_speed_deg_s=homing_cfg.get("speed_deg_s", 5.0),
        homing_tolerance_deg=homing_cfg.get("tolerance_deg", 1.0),
        nudge_speed_deg_s=cfg.get("nudge", {}).get("speed_deg_s", 15.0),
        joints=joints,
    )
