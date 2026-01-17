# utilities.mk - Debug and maintenance utility targets
# Collection of helpful tools for debugging, testing, and maintenance

# ============================================================
# Device Information & Debugging
# ============================================================

.PHONY: efuse
efuse: check_tools
	espefuse.py --port $(port) summary

.PHONY: hash
hash: check_tools
	@esptool.py --chip $(CHIP) image_info $(BUILD_DIR)/IonBridge.bin | tail -n 1 | cut -d' ' -f 3
	@ls -alt $(BUILD_DIR)/IonBridge.bin | tail -n 1 | awk '{print $$5}'

# ============================================================
# NVS & Flash Management
# ============================================================

.PHONY: erase_nvs
erase_nvs: check_tools
	@dd if=/dev/zero of=/tmp/nvs_empty.bin bs=16k count=1
	esptool.py --no-stub --chip $(CHIP) -p $(port) -b $(BAUDRATE) \
		--before=default_reset --after=hard_reset write_flash \
		--flash_mode $(FLASH_MODE) --flash_freq $(FLASH_FREQ) --flash_size $(FLASH_SIZE) \
		--force $(NVS_DATA_OFFSET) /tmp/nvs_empty.bin

.PHONY: decrypt_flash_data
decrypt_flash_data: check_tools
	@if [ -z "$(ADDRESS)" ] || [ -z "$(SIZE)" ]; then \
		echo "Error: ADDRESS and SIZE parameters required"; \
		echo "Usage: make decrypt_flash_data ADDRESS=0x10000 SIZE=0x100000"; \
		exit 1; \
	fi
	esptool.py --no-stub --chip $(CHIP) -p $(port) -b $(BAUDRATE) \
		read_flash $(ADDRESS) $(SIZE) /tmp/flash_encrypted_data.bin
	espsecure.py decrypt_flash_data --keyfile $(FLASH_ENCRYPT_KEY) \
		/tmp/flash_encrypted_data.bin --output flash_data.bin -a $(ADDRESS)

# ============================================================
# NVS Key Management
# ============================================================

$(SECURE_NVS_KEY_BIN):
	@mkdir -p $(dir $@)
	$(IDF_PATH)/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py \
		generate-key --keyfile $@

.PHONY: gen_nvs_key
gen_nvs_key: $(SECURE_NVS_KEY_BIN)

.PHONY: encrypt_nvs_key
encrypt_nvs_key: gen_nvs_key
	@if [ -z "$(ADDRESS)" ]; then \
		echo "Error: ADDRESS parameter required"; \
		echo "Usage: make encrypt_nvs_key ADDRESS=0x3F5000"; \
		exit 1; \
	fi
	espsecure.py encrypt_flash_data --aes_xts --keyfile $(FLASH_ENCRYPT_KEY) \
		--address $(ADDRESS) -o ./keys/encrypted_nvs_keys.bin $(SECURE_NVS_KEY_BIN)

.PHONY: update_nvs_key
update_nvs_key: check_tools
	@if [ -z "$(ADDRESS)" ]; then \
		echo "Error: ADDRESS parameter required"; \
		echo "Usage: make update_nvs_key ADDRESS=0x3F5000"; \
		exit 1; \
	fi
	@$(MAKE) encrypt_nvs_key ADDRESS=$(ADDRESS)
	esptool.py --no-stub --chip $(CHIP) -p $(port) -b $(BAUDRATE) \
		--before=default_reset --after=hard_reset write_flash \
		--flash_mode $(FLASH_MODE) --flash_freq $(FLASH_FREQ) --flash_size $(FLASH_SIZE) \
		--force $(ADDRESS) ./keys/encrypted_nvs_keys.bin

# ============================================================
# Build Artifacts & Packaging
# ============================================================

.PHONY: release_tar
release_tar: build
	@mkdir -p build/mytar
	@cd build/mytar && \
		cp ../bootloader/bootloader.bin . && \
		cp ../IonBridge.bin . && \
		cp ../partition_table/partition-table.bin . && \
		cp ../ota_data_initial.bin .
	@cd build/mytar && tar -cvf /tmp/esp32.tar .
	@mv /tmp/esp32.tar build/esp32.tar
	@rm -rf build/mytar
	@echo "Created: build/esp32.tar"
