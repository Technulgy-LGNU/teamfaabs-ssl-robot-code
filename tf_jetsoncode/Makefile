SHELL := /usr/bin/env bash
.DEFAULT_GOAL := cross-build-debug

TARGET := aarch64-unknown-linux-gnu
BINARY := tf_jetsoncode
CROSS ?= cross

SSH_USER ?= robotik
ROBOT ?= all

# Add robots here as needed. Use the robot id in ROBOT_IDS and define
# ROBOT_<id>_HOST for its SSH host/IP.
ROBOT_IDS ?= 0 1 2 4 5
ROBOT_0_HOST ?= 10.0.64.100
ROBOT_1_HOST ?= 10.0.64.101
ROBOT_2_HOST ?= 10.0.64.102
ROBOT_3_HOST ?= 10.0.64.103
ROBOT_4_HOST ?= 10.0.64.104
ROBOT_5_HOST ?= 10.0.64.105

REMOTE_DIR ?= /home/robotik/tf_jetsoncode
DEBUG_REMOTE_DIR ?= /home/robotik/Documents/teamfaabs-ssl-robot-code/tf_jetsoncode
DEBUG_RUN_DIR ?= /home/robotik/tf_jetsoncode/tf_jetsoncode

DEBUG_BIN := target/$(TARGET)/debug/$(BINARY)
RELEASE_BIN := target/$(TARGET)/release/$(BINARY)

define robot_host_case
$(1)) host="$($(strip ROBOT_$(1)_HOST))" ;;
endef
ROBOT_HOST_CASES := $(foreach id,$(ROBOT_IDS),$(call robot_host_case,$(id)))

ifeq ($(ROBOT),all)
SELECTED_ROBOTS := $(ROBOT_IDS)
else
SELECTED_ROBOTS := $(ROBOT)
endif

ROBOT_TARGETS := upload-debug upload-release run-debug run-release
ifneq ($(filter $(ROBOT_TARGETS),$(MAKECMDGOALS)),)
UNKNOWN_ROBOTS := $(strip $(filter-out $(ROBOT_IDS),$(SELECTED_ROBOTS)))
MISSING_ROBOT_HOSTS := $(strip $(foreach id,$(SELECTED_ROBOTS),$(if $($(strip ROBOT_$(id)_HOST)),,$(id))))
ifneq ($(UNKNOWN_ROBOTS),)
$(error Unknown ROBOT "$(UNKNOWN_ROBOTS)"; known robots: $(ROBOT_IDS). Use ROBOT=all or add ROBOT_<id>_HOST)
endif
ifneq ($(MISSING_ROBOT_HOSTS),)
$(error Missing host for robot(s) "$(MISSING_ROBOT_HOSTS)"; define ROBOT_<id>_HOST)
endif
endif

.PHONY: help list-robots cross-build-debug cross-build-release upload-debug upload-release run-debug run-release run-deploy-release

help:
	@echo "Usage:"
	@echo "  make upload-debug ROBOT=2"
	@echo "  make upload-release ROBOT=all"
	@echo "  make run-debug ROBOT=2"
	@echo "  make run-release ROBOT=2"
	@echo "  make run-deploy-release ROBOT=all"
	@echo ""
	@echo "Configured robots:"
	@$(MAKE) --no-print-directory list-robots

list-robots:
	@$(foreach id,$(ROBOT_IDS),echo "  $(id) -> $($(strip ROBOT_$(id)_HOST))";)

cross-build-release:
	$(CROSS) build --target $(TARGET) --release

upload-release: cross-build-release
	@set -e; \
	for id in $(SELECTED_ROBOTS); do \
		case "$$id" in $(ROBOT_HOST_CASES) *) echo "Unknown robot $$id"; exit 1 ;; esac; \
		echo "==> Uploading release to robot $$id ($$host)"; \
		ssh $(SSH_USER)@$$host "pkill $(BINARY) || true"; \
		scp $(RELEASE_BIN) $(SSH_USER)@$$host:$(REMOTE_DIR); \
	done

run-release: upload-release
	@set -e; \
	for id in $(SELECTED_ROBOTS); do \
		case "$$id" in $(ROBOT_HOST_CASES) *) echo "Unknown robot $$id"; exit 1 ;; esac; \
		echo "==> Running release on robot $$id ($$host)"; \
		ssh $(SSH_USER)@$$host "cd $(REMOTE_DIR) && ./$(BINARY)"; \
	done

run-deploy-release: cross-build-release
	@set -e; \
	for id in $(SELECTED_ROBOTS); do \
		case "$$id" in $(ROBOT_HOST_CASES) *) echo "Unknown robot $$id"; exit 1 ;; esac; \
        echo "==> Uploading release to robot $$id ($$host)"; \
        ssh $(SSH_USER)@$$host "sudo systemctl stop tf_jetsoncode.service || true"; \
        scp $(RELEASE_BIN) $(SSH_USER)@$$host:$(REMOTE_DIR); \
        ssh $(SSH_USER)@$$host "sudo systemctl start tf_jetsoncode.service || true"; \
	done