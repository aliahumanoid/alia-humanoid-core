## Switchable dev runner for software stack (host + firmware)

# Tools
PY ?= python3
PIP ?= pip3
PIO ?= pio

# Paths
ROOT := $(abspath ..)
HOST_DIR := $(CURDIR)/host
FW_CONTROLLER_DIR := $(CURDIR)/firmware/joint_controller
FW_ENCODERS_DIR := $(CURDIR)/firmware/joint_encoders
SCRIPTS_DIR := $(CURDIR)/scripts
JOINT_CONFIG_JSON := $(CURDIR)/joint_config.json
CONFIG_PRESETS_H := $(FW_CONTROLLER_DIR)/include/config_presets.h

# Defaults
ENV ?= pico
MODE ?= host

.PHONY: help
help:
	@echo "Targets:"
	@echo "  make run MODE=host                 # run Python host app"
	@echo "  make host-venv|host-install|host-run|host-sanity"
	@echo "  make run MODE=firmware-controller  ENV=pico|pico_debug"
	@echo "  make run MODE=firmware-encoders    ENV=rpipico2|rpipico2_debug"
	@echo "  make fw-build-controller|fw-build-encoders"
	@echo "  make fw-monitor-controller|fw-monitor-encoders [PORT=/dev/tty*]"
	@echo "  make joint-config                  # generate joint_config.json from firmware"

.PHONY: run
run:
ifeq ($(MODE),host)
	$(MAKE) host-run
else ifeq ($(MODE),firmware-controller)
	$(MAKE) fw-build-controller ENV=$(ENV)
else ifeq ($(MODE),firmware-encoders)
	$(MAKE) fw-build-encoders ENV=$(ENV)
else
	@echo "Unknown MODE=$(MODE). Use host|firmware-controller|firmware-encoders"; exit 2
endif

# ---------------- Joint Configuration Sync ----------------
# Generate joint_config.json from firmware config_presets.h
# This ensures firmware and host stay synchronized
.PHONY: joint-config
joint-config:
	@echo "Generating joint_config.json from firmware..."
	@"$(PY)" "$(SCRIPTS_DIR)/extract_joint_config.py" "$(CONFIG_PRESETS_H)" "$(JOINT_CONFIG_JSON)"

# Auto-generate joint config if source file is newer
$(JOINT_CONFIG_JSON): $(CONFIG_PRESETS_H)
	@echo "Config file out of date, regenerating..."
	@$(MAKE) joint-config

# ---------------- Host (Python) ----------------
HOST_PY := $(HOST_DIR)/.venv/bin/python
HOST_PIP := $(HOST_DIR)/.venv/bin/pip

.PHONY: host-venv
host-venv:
	@test -d "$(HOST_DIR)/.venv" || (cd "$(HOST_DIR)" && "$(PY)" -m venv .venv)

.PHONY: host-install
host-install: host-venv
	cd "$(HOST_DIR)" && "$(HOST_PIP)" install --upgrade pip
	cd "$(HOST_DIR)" && "$(HOST_PIP)" install -r requirements.txt

.PHONY: host-sanity
host-sanity: host-install
	cd "$(HOST_DIR)" && "$(HOST_PY)" sanity_check.py

.PHONY: host-run
host-run: host-install $(JOINT_CONFIG_JSON)
	cd "$(HOST_DIR)" && FLASK_DEBUG=$${FLASK_DEBUG:-False} "$(HOST_PY)" main.py

# ---------------- Firmware (PlatformIO) ----------------
.PHONY: fw-build-controller
fw-build-controller:
	cd "$(FW_CONTROLLER_DIR)" && "$(PIO)" run -e $(ENV)

.PHONY: fw-build-encoders
fw-build-encoders:
	cd "$(FW_ENCODERS_DIR)" && "$(PIO)" run -e $(ENV)

.PHONY: fw-monitor-controller
fw-monitor-controller:
	@PORT=$${PORT:?"Set PORT=/dev/tty..."}; cd "$(FW_CONTROLLER_DIR)" && "$(PIO)" device monitor -p $$PORT -b 115200

.PHONY: fw-monitor-encoders
fw-monitor-encoders:
	@PORT=$${PORT:?"Set PORT=/dev/tty..."}; cd "$(FW_ENCODERS_DIR)" && "$(PIO)" device monitor -p $$PORT -b 115200
