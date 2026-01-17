# config.mk - Centralized configuration and parameter defaults
# All variables, paths, and parameter-based configuration

# ============================================================
# Directory Paths (needed early for sticky variant)
# ============================================================

BUILD_DIR := $(CURDIR)/build

# ============================================================
# Default Parameters
# ============================================================

# Read saved variant from build directory if not explicitly set on command line
SAVED_VARIANT_FILE := $(BUILD_DIR)/.variant
SAVED_BUILD_PARAMS_FILE := $(BUILD_DIR)/.build_params
ifndef variant
    ifneq ($(wildcard $(SAVED_VARIANT_FILE)),)
        variant := $(shell cat $(SAVED_VARIANT_FILE) 2>/dev/null)
    endif
endif

# Build parameters (can be overridden: make build variant=3536 secure=1)
variant ?= cp02
coredump ?= 1

# Detect variant change (compare saved vs current)
SAVED_VARIANT := $(shell cat $(SAVED_VARIANT_FILE) 2>/dev/null)
VARIANT_CHANGED := $(if $(SAVED_VARIANT),$(if $(filter $(SAVED_VARIANT),$(variant)),,1),)

# Check sdkconfig once for secure boot (used by both production and secure detection)
_SDKCONFIG_HAS_SECURE_BOOT := $(if $(wildcard sdkconfig),$(shell grep -q "^CONFIG_SECURE_BOOT=y" sdkconfig 2>/dev/null && echo 1))
_VARIANT_FROM_CLI := $(filter command line,$(origin variant))

# Determine if sdkconfig refresh is needed
NEEDS_SDKCONFIG_REFRESH := $(strip $(or \
    $(_VARIANT_FROM_CLI), \
    $(FORCE_SDKCONFIG), \
    $(VARIANT_CHANGED), \
    $(if $(wildcard sdkconfig),,1)))

# Auto-detect production: development if switching variants, otherwise check sdkconfig
ifndef production
    production := $(if $(_VARIANT_FROM_CLI),0,$(_SDKCONFIG_HAS_SECURE_BOOT))
endif
production ?= 0

# Auto-detect secure: follows production (secure boot config implies both)
ifndef secure
    secure := $(production)
endif
secure ?= 0

# Default environment variables for ESP-IDF component manager
# These are required by idf_component.yml conditional dependencies
USE_LVGL ?= 0
USE_GZIP_COMPRESSION ?= 1
USE_XZ_COMPRESSION ?= 0
export USE_LVGL USE_GZIP_COMPRESSION USE_XZ_COMPRESSION

# Unlocked firmware flag (unlock=1 implies littlefs=1)
unlock ?= 0
export unlock

# Flash parameters (can be overridden: make flash littlefs=1 protected_data=1)
ifeq ($(unlock),1)
    littlefs ?= 1
else
    littlefs ?= 0
endif
protected_data ?= 0
nvs ?= 0
phy_init ?= 0

# ============================================================
# Hardware Configuration
# ============================================================

port ?= /dev/ttyUSB0
CHIP := esp32c3
BAUDRATE := 460800
FLASH_MODE := dio
FLASH_FREQ := 80m
FLASH_SIZE := 4MB

# Security keys
FLASH_ENCRYPT_KEY := my_flash_encryption_key.bin

# PSN shortcut: psn=1124703503552092 → sign_key=sign_keys/1124703503552092_sign_key.pem
ifdef psn
    sign_key := sign_keys/$(psn)_sign_key.pem
endif

# ============================================================
# Directory Paths (continued)
# ============================================================

COMPONENTS_DIR := $(CURDIR)/components
LITTLEFS_DIR := $(CURDIR)/littlefs
FS_DIR := $(CURDIR)/files
CFG_DIR := $(CURDIR)/configs

# ============================================================
# Variant Configuration
# ============================================================

# Variant to sdkconfig mapping
variant_config_cp02 := develop
variant_config_cp02s := cp02s.develop
variant_config_3536 := 3536.develop
variant_config_gpu_demo := develop
variant_config_bird := bird

# Variant environment variables
variant_env_cp02 := USE_GZIP_COMPRESSION=1 USE_XZ_COMPRESSION=0 USE_LVGL=0
variant_env_cp02s := USE_GZIP_COMPRESSION=1 USE_XZ_COMPRESSION=0 USE_LVGL=0
variant_env_3536 := USE_GZIP_COMPRESSION=1 USE_XZ_COMPRESSION=0 USE_LVGL=1
variant_env_gpu_demo := USE_GZIP_COMPRESSION=1 USE_XZ_COMPRESSION=0 USE_LVGL=0
variant_env_bird := USE_GZIP_COMPRESSION=1 USE_XZ_COMPRESSION=0 USE_LVGL=0

# Variant to product family mapping
variant_family_cp02 := CP02
variant_family_cp02s := CP02S
variant_family_3536 := PA768S
variant_family_gpu_demo := CP02
variant_family_bird := bird

# Set PRODUCT_FAMILY based on variant (override the default)
PRODUCT_FAMILY := $(or $(variant_family_$(variant)),CP02)

# Determine sdkconfig source file
ifeq ($(production),1)
    ifneq ($(wildcard $(CFG_DIR)/sdkconfig-$(variant).production),)
        SDKCONFIG_SRC := $(CFG_DIR)/sdkconfig-$(variant).production
    else
        SDKCONFIG_SRC := $(CFG_DIR)/sdkconfig.production
    endif
else
    SDKCONFIG_SRC := $(CFG_DIR)/sdkconfig.$(variant_config_$(variant))
endif

# ============================================================
# Binary Paths and Offsets (read from sdkconfig/partition CSV)
# ============================================================

# Helper: read config value from sdkconfig
# Usage: $(call get_config,CONFIG_NAME)
get_config = $(shell grep "^$(1)=" sdkconfig 2>/dev/null | cut -d= -f2 | tr -d '"')

# Helper: get partition offset from CSV
# Usage: $(call get_partition_offset,partition_name)
get_partition_offset = $(shell awk -F',' '/^$(1),/{gsub(/[[:space:]]/,"",$$4); print $$4}' $(PARTITION_CSV) 2>/dev/null)

# Helper: get partition size from CSV
# Usage: $(call get_partition_size,partition_name)
get_partition_size = $(shell awk -F',' '/^$(1),/{gsub(/[[:space:]]/,"",$$5); print $$5}' $(PARTITION_CSV) 2>/dev/null)

# Partition table from sdkconfig
PARTITION_TABLE_OFFSET := $(or $(call get_config,CONFIG_PARTITION_TABLE_OFFSET),0xc000)
PARTITION_CSV := $(or $(call get_config,CONFIG_PARTITION_TABLE_CUSTOM_FILENAME),partitions/release-partitions.csv)

# Bootloader
BOOTLOADER_OFFSET := 0x0
BOOTLOADER_BIN := $(BUILD_DIR)/bootloader/bootloader.bin

# Partition table
PARTITION_TABLE_BIN := $(BUILD_DIR)/partition_table/partition-table.bin

# OTA data (from partition CSV)
OTA_DATA_INITIAL_OFFSET := $(or $(call get_partition_offset,otadata),0xe000)
OTA_DATA_INITIAL_BIN := $(BUILD_DIR)/ota_data_initial.bin

# Application
APP_OFFSET := 0x10000
APP_BIN := $(BUILD_DIR)/IonBridge.bin
BOOTABLE_APP_BIN := $(BUILD_DIR)/bootable.bin

# LittleFS partition (from partition CSV)
LITTLEFS_OFFSET := $(or $(call get_partition_offset,littlefs),0x330000)
LITTLEFS_LEN := $(or $(call get_partition_size,littlefs),0xC5000)

# NVS key partition
SECURE_NVS_KEY_OFFSET := $(or $(call get_partition_offset,nvs_key),0x3f5000)

# NVS data partition
NVS_DATA_OFFSET := $(or $(call get_partition_offset,nvs),0x3F6000)
NVS_DATA_LEN := $(or $(call get_partition_size,nvs),0x4000)

# PHY init partition
PHY_INIT_OFFSET := $(or $(call get_partition_offset,phy_init),0x3FA000)

# Protected data partition
PROTECTED_DATA_OFFSET := $(or $(call get_partition_offset,protected_data),0x3FB000)
PROTECTED_DATA_LEN := $(or $(call get_partition_size,protected_data),0x5000)

# Signed binaries (for secure boot)
SIGNED_BOOTLOADER_BIN := $(BUILD_DIR)/bootloader_signed.bin
SIGNED_APP_BIN := $(BUILD_DIR)/IonBridge_signed.bin

# Partitions
LITTLEFS_BIN := $(BUILD_DIR)/littlefs.bin
PROTECTED_DATA_CSV := $(CURDIR)/protected_data.csv
PROTECTED_DATA_BIN := $(BUILD_DIR)/protected_data.bin
NVS_DATA_CSV := $(CURDIR)/nvs_data.csv
NVS_DATA_BIN := $(BUILD_DIR)/nvs_data.bin
PHY_INIT_BIN := $(CURDIR)/phy_init.bin

# Output binaries
OUTPUT_BIN := $(BUILD_DIR)/output.bin

# Secure boot keys
SECURE_NVS_KEY_BIN := $(CURDIR)/keys/nvs_keys.bin
