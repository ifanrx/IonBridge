# flash.mk - Parameter-based flash logic
# Handles conditional partition flashing based on parameters

# ============================================================
# Partition Generation
# ============================================================

.PHONY: gen_littlefs
gen_littlefs: check_tools prepare_littlefs_dir
	$(call create_littlefs)

.PHONY: gen_protected_data
gen_protected_data: check_tools
	@if [ "$(secure)" = "1" ] && [ -f "$(SECURE_NVS_KEY_BIN)" ]; then \
		$(IDF_PATH)/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py \
			encrypt --inputkey $(SECURE_NVS_KEY_BIN) \
			$(PROTECTED_DATA_CSV) $(PROTECTED_DATA_BIN) $(PROTECTED_DATA_LEN); \
	else \
		$(IDF_PATH)/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py \
			generate $(PROTECTED_DATA_CSV) $(PROTECTED_DATA_BIN) $(PROTECTED_DATA_LEN); \
	fi

.PHONY: gen_nvs_data
gen_nvs_data: check_tools
	$(eval NVS_CSV ?= $(NVS_DATA_CSV))
	$(IDF_PATH)/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py \
		generate $(NVS_CSV) $(NVS_DATA_BIN) $(NVS_DATA_LEN)

# ============================================================
# Filesystem Preparation
# ============================================================

# Helper function to prepare unlocked firmware
define prepare_unlocked_firmware
	@rm -rf unlocked_fs && mkdir -p unlocked_fs
	@if [ -d "$(FS_DIR)/$(1)/littlefs" ]; then \
		cp -r $(FS_DIR)/$(1)/littlefs/* unlocked_fs/; \
	fi
	@for f in $(FS_DIR)/$(1)/factory/SW3566_*_unlocked.bin; do \
		[ -f "$$f" ] && cp "$$f" unlocked_fs/$$(basename "$$f" | sed 's/_unlocked//') && \
		echo "Copied unlocked firmware: $$(basename $$f)"; \
	done
endef

# Sentinel file to track if littlefs dir was already prepared in this session
LITTLEFS_SENTINEL := $(BUILD_DIR)/.littlefs_prepared

.PHONY: prepare_littlefs_dir
prepare_littlefs_dir: prepare_sdkconfig
	@# Skip if already prepared in this session (prevents double gzip with different SHA-1)
	@if [ -f "$(LITTLEFS_SENTINEL)" ] && [ -d "$(LITTLEFS_DIR)" ]; then \
		echo "Littlefs directory already prepared, skipping"; \
		exit 0; \
	fi
	@echo "Product family: $(PRODUCT_FAMILY)"
	@rm -rf $(LITTLEFS_DIR) unlocked_fs 2>/dev/null || true
	@mkdir -p $(LITTLEFS_DIR)
	@# Determine source directory
	@if [ "$(unlock)" = "1" ]; then \
		mkdir -p unlocked_fs; \
		if [ -d "$(FS_DIR)/$(PRODUCT_FAMILY)/littlefs" ]; then \
			cp -r $(FS_DIR)/$(PRODUCT_FAMILY)/littlefs/* unlocked_fs/; \
		fi; \
		for f in $(FS_DIR)/$(PRODUCT_FAMILY)/factory/SW3566_*_unlocked.bin; do \
			[ -f "$$f" ] && cp "$$f" unlocked_fs/$$(basename "$$f" | sed 's/_unlocked//') && \
			echo "Copied unlocked firmware: $$(basename $$f)"; \
		done; \
		SRC_DIR="unlocked_fs"; \
	elif [ -d "$(FS_DIR)/$(PRODUCT_FAMILY)/littlefs" ]; then \
		SRC_DIR="$(FS_DIR)/$(PRODUCT_FAMILY)/littlefs"; \
	else \
		echo "No littlefs directory found for $(PRODUCT_FAMILY), using empty directory"; \
		exit 0; \
	fi; \
	echo "Source directory: $$SRC_DIR"; \
	if [ -n "$(USE_XZ_COMPRESSION)" ] && [ "$(USE_XZ_COMPRESSION)" = "1" ]; then \
		echo "Compressing firmware files with XZ (from make parameter)..."; \
		COMPRESS_METHOD="xz"; \
	elif [ -n "$(USE_GZIP_COMPRESSION)" ] && [ "$(USE_GZIP_COMPRESSION)" = "1" ]; then \
		echo "Compressing firmware files with GZIP (from make parameter)..."; \
		COMPRESS_METHOD="gzip"; \
	else \
		eval $(variant_env_$(variant)); \
		if [ "$$USE_XZ_COMPRESSION" = "1" ]; then \
			echo "Compressing firmware files with XZ (from variant env)..."; \
			COMPRESS_METHOD="xz"; \
		elif [ "$$USE_GZIP_COMPRESSION" = "1" ]; then \
			echo "Compressing firmware files with GZIP (from variant env)..."; \
			COMPRESS_METHOD="gzip"; \
		elif [ -f sdkconfig ] && grep -q "^CONFIG_STORAGE_COMPRESSION_XZ=y" sdkconfig 2>/dev/null; then \
			echo "Compressing firmware files with XZ (from sdkconfig)..."; \
			COMPRESS_METHOD="xz"; \
		elif [ -f sdkconfig ] && grep -q "^CONFIG_STORAGE_COMPRESSION_GZIP=y" sdkconfig 2>/dev/null; then \
			echo "Compressing firmware files with GZIP (from sdkconfig)..."; \
			COMPRESS_METHOD="gzip"; \
		else \
			echo "No compression configured, copying files as-is"; \
			COMPRESS_METHOD="none"; \
		fi; \
	fi; \
	if [ "$$COMPRESS_METHOD" = "xz" ]; then \
		for f in $$SRC_DIR/*; do \
			if [ -d "$$f" ]; then \
				echo "  Copying directory $$(basename $$f)/"; \
				cp -r "$$f" "$(LITTLEFS_DIR)/"; \
			elif [ -f "$$f" ]; then \
				fname=$$(basename "$$f"); \
				if echo "$$fname" | grep -qE '^(FPGA_|SW3566_).*\.bin$$'; then \
					echo "  Compressing $$fname -> $$fname.xz"; \
					xz -0 --lzma2=dict=4KiB --check=crc32 -k -c "$$f" > "$(LITTLEFS_DIR)/$$fname.xz"; \
				else \
					cp "$$f" "$(LITTLEFS_DIR)/"; \
				fi; \
			fi; \
		done; \
	elif [ "$$COMPRESS_METHOD" = "gzip" ]; then \
		if [ -d "venv" ]; then \
			python_cmd="$${PWD}/venv/bin/python3"; \
		else \
			python_cmd="python3"; \
		fi; \
		for f in $$SRC_DIR/*; do \
			if [ -d "$$f" ]; then \
				echo "  Copying directory $$(basename $$f)/"; \
				cp -r "$$f" "$(LITTLEFS_DIR)/"; \
			elif [ -f "$$f" ]; then \
				fname=$$(basename "$$f"); \
				if echo "$$fname" | grep -qE '^(FPGA_|SW3566_).*\.bin$$'; then \
					$$python_cmd python/update_version_cfg.py compress "$$f" "$(LITTLEFS_DIR)/$$fname.gz"; \
				else \
					cp "$$f" "$(LITTLEFS_DIR)/"; \
				fi; \
			fi; \
		done; \
	else \
		cp -r $$SRC_DIR/* $(LITTLEFS_DIR)/ 2>/dev/null || true; \
	fi
	@# Mark as prepared to prevent double-run in same session (only if build dir exists)
	@if [ -d "$(BUILD_DIR)" ]; then touch $(LITTLEFS_SENTINEL); fi

# Legacy alias for backwards compatibility
.PHONY: link_littlefs_dir
link_littlefs_dir: prepare_littlefs_dir

define create_littlefs
	$(eval LITTLEFS_VER := $(shell littlefs-python --version 2>&1 | grep -o '[0-9.]\+' | head -n 1))
	$(eval LITTLEFS_SIZE := $(LITTLEFS_LEN))
	@if [ -n "$(LITTLEFS_VER)" ] && \
		([ $$(echo $(LITTLEFS_VER) | cut -d. -f1) -gt 0 ] || \
		([ $$(echo $(LITTLEFS_VER) | cut -d. -f1) -eq 0 ] && \
		[ $$(echo $(LITTLEFS_VER) | cut -d. -f2) -gt 13 ])); then \
			littlefs-python create -v --fs-size=$(LITTLEFS_SIZE) --name-max=64 --block-size=4096 $(LITTLEFS_DIR) $(LITTLEFS_BIN); \
	else \
	    littlefs-python create -v --image=$(LITTLEFS_BIN) --fs-size=$(LITTLEFS_SIZE) --name-max=64 --block-size=4096 $(LITTLEFS_DIR); \
	fi
endef

# ============================================================
# Main Flash Target (Parameter-Based)
# ============================================================

# Check if rebuild needed using ninja dry-run + build params check
.PHONY: check_rebuild
check_rebuild:
	@# Clean sentinel to ensure fresh preparation each flash session
	@rm -f $(LITTLEFS_SENTINEL)
	@# Step 1: Update version.json with checksums (runs prepare_littlefs_dir via dependency)
	@if [ "$(littlefs)" = "1" ]; then \
		$(MAKE) update_version; \
	fi
	@# Step 2: Check if rebuild needed and build
	@needs_build=0; \
	if [ ! -f "$(APP_BIN)" ]; then \
		needs_build=1; \
	elif [ -f "$(SAVED_BUILD_PARAMS_FILE)" ]; then \
		saved_params=$$(cat "$(SAVED_BUILD_PARAMS_FILE)"); \
		if [ "$$saved_params" != "$(CURRENT_BUILD_PARAMS)" ]; then \
			echo "Build parameters changed, rebuild needed"; \
			needs_build=1; \
		fi; \
	fi; \
	if [ "$$needs_build" = "0" ] && [ -f sdkconfig ] && [ -d "$(BUILD_DIR)" ]; then \
		sdk_gzip=$$(grep -q "^CONFIG_STORAGE_COMPRESSION_GZIP=y" sdkconfig 2>/dev/null && echo 1 || echo 0); \
		sdk_xz=$$(grep -q "^CONFIG_STORAGE_COMPRESSION_XZ=y" sdkconfig 2>/dev/null && echo 1 || echo 0); \
		if [ "$$sdk_gzip" != "$(USE_GZIP_COMPRESSION)" ] || [ "$$sdk_xz" != "$(USE_XZ_COMPRESSION)" ]; then \
			echo "sdkconfig out of sync, rebuild needed"; \
			needs_build=1; \
		fi; \
	fi; \
	if [ "$$needs_build" = "0" ] && [ "$(littlefs)" = "1" ] && [ -f sdkconfig ]; then \
		current_sha1=$$(grep "^CONFIG_SW3566_FILE_SHA1=" sdkconfig 2>/dev/null | cut -d= -f2 | tr -d '"'); \
		if [ -f "$(BUILD_DIR)/.last_checksum" ]; then \
			last_sha1=$$(cat "$(BUILD_DIR)/.last_checksum" 2>/dev/null); \
			if [ "$$current_sha1" != "$$last_sha1" ]; then \
				echo "Firmware checksum changed ($$last_sha1 -> $$current_sha1), rebuild needed"; \
				needs_build=1; \
			fi; \
		else \
			echo "No saved checksum found, rebuild needed for littlefs"; \
			needs_build=1; \
		fi; \
	fi; \
	if [ "$$needs_build" = "0" ] && [ -d "$(BUILD_DIR)" ]; then \
		if ! ninja -C $(BUILD_DIR) -n 2>/dev/null | grep -q "no work to do"; then \
			needs_build=1; \
		fi; \
	fi; \
	if [ "$$needs_build" = "1" ]; then \
		$(MAKE) build; \
	fi

.PHONY: flash
flash: check_rebuild
	@echo "========================================"
	@echo "Flash Configuration"
	@echo "========================================"
	@echo "  variant:        $(variant)"
	@echo "  product_family: $(PRODUCT_FAMILY)"
	@echo "  secure:         $(secure)"
	@echo "  port:           $(port)"
	@if [ -n "$(psn)" ]; then \
		echo "  psn:            $(psn)"; \
	elif [ -n "$(sign_key)" ]; then \
		echo "  sign_key:       $(sign_key)"; \
	fi
	@echo "  littlefs:       $(littlefs)"
	@echo "  protected_data: $(protected_data)"
	@echo "  nvs:            $(nvs)"
	@echo "  phy_init:       $(phy_init)"
	@echo "========================================"
	@sdk_secure=$$(grep -q "^CONFIG_SECURE_BOOT=y" sdkconfig 2>/dev/null && echo 1 || echo 0); \
	if [ "$(secure)" = "1" ] && [ "$$sdk_secure" != "1" ]; then \
		echo "Error: secure=1 specified but firmware was built without CONFIG_SECURE_BOOT=y"; \
		echo "  Rebuild with: make build variant=$(variant) production=1"; \
		exit 1; \
	fi
	@if [ "$(littlefs)" = "1" ]; then \
		$(MAKE) flash_littlefs_partition; \
	fi
	@if [ "$(protected_data)" = "1" ]; then \
		$(MAKE) flash_protected_data_partition; \
	fi
	@if [ "$(nvs)" = "1" ]; then \
		$(MAKE) flash_nvs_partition; \
	fi
	@if [ "$(phy_init)" = "1" ]; then \
		$(MAKE) flash_phy_init_partition; \
	fi
	@$(MAKE) flash_firmware

# ============================================================
# Firmware Flash (Base)
# ============================================================

.PHONY: flash_firmware
flash_firmware: $(APP_BIN)
	@if [ "$(secure)" = "1" ]; then \
		if [ -z "$(sign_key)" ]; then \
			echo "Error: Secure flash requires sign_key or psn parameter"; \
			echo "  Usage: make flash psn=1124703503552092"; \
			echo "     or: make flash sign_key=path/to/key.pem"; \
			exit 1; \
		fi; \
		if [ ! -f "$(sign_key)" ]; then \
			echo "Error: Signing key not found: $(sign_key)"; \
			exit 1; \
		fi; \
		echo "Signing binaries for secure boot..."; \
		$(call sign_binary,$(APP_BIN),$(SIGNED_APP_BIN)); \
		echo "Flashing signed firmware with encryption..."; \
		$(call write_flash_encrypted_force,\
			$(PARTITION_TABLE_OFFSET) $(PARTITION_TABLE_BIN) \
			$(OTA_DATA_INITIAL_OFFSET) $(OTA_DATA_INITIAL_BIN) \
			$(APP_OFFSET) $(SIGNED_APP_BIN)); \
	else \
		idf.py -p $(port) flash; \
	fi

# ============================================================
# Individual Partition Flash
# ============================================================

.PHONY: flash_littlefs_partition
flash_littlefs_partition: gen_littlefs
	@if [ "$(secure)" = "1" ]; then \
		$(call write_flash_encrypted,$(LITTLEFS_OFFSET) $(LITTLEFS_BIN)); \
	else \
		$(call write_flash,$(LITTLEFS_OFFSET) $(LITTLEFS_BIN)); \
	fi

.PHONY: flash_protected_data_partition
flash_protected_data_partition: gen_protected_data
	@if [ "$(secure)" = "1" ]; then \
		$(call write_flash_force,$(PROTECTED_DATA_OFFSET) $(PROTECTED_DATA_BIN)); \
	else \
		$(call write_flash,$(PROTECTED_DATA_OFFSET) $(PROTECTED_DATA_BIN)); \
	fi

.PHONY: flash_nvs_partition
flash_nvs_partition: gen_nvs_data
	$(call write_flash,$(NVS_DATA_OFFSET) $(NVS_DATA_BIN))

.PHONY: flash_phy_init_partition
flash_phy_init_partition: check_tools
	$(call write_flash_force,$(PHY_INIT_OFFSET) $(PHY_INIT_BIN))

.PHONY: flash_partition_table
flash_partition_table: check_tools
	@if [ "$(secure)" = "1" ]; then \
		$(call write_flash_encrypted_force,$(PARTITION_TABLE_OFFSET) $(PARTITION_TABLE_BIN)); \
	else \
		$(call write_flash,$(PARTITION_TABLE_OFFSET) $(PARTITION_TABLE_BIN)); \
	fi
