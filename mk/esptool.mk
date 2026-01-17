# esptool.mk - Wrapper functions for esptool.py and espsecure.py
# Eliminates duplication of common esptool commands throughout the Makefile

# Merge binary images into a single flash image
# Usage: $(call merge_flash_bin,OUTPUT_FILE,OFFSET1 BIN1 OFFSET2 BIN2 ...)
define merge_flash_bin
	esptool.py --chip $(CHIP) merge_bin \
		--output $(1) --format raw \
		--flash_freq $(FLASH_FREQ) --flash_mode $(FLASH_MODE) --flash_size $(FLASH_SIZE) \
		$(2)
endef

# Merge binary with --fill-flash-size option
# Usage: $(call merge_flash_bin_fill,OUTPUT_FILE,OFFSET1 BIN1 OFFSET2 BIN2 ...)
define merge_flash_bin_fill
	esptool.py --chip $(CHIP) merge_bin \
		--output $(1) --format raw \
		--flash_freq $(FLASH_FREQ) --flash_mode $(FLASH_MODE) --flash_size $(FLASH_SIZE) \
		--fill-flash-size $(FLASH_SIZE) \
		$(2)
endef

# Write flash (non-encrypted)
# Usage: $(call write_flash,OFFSET1 BIN1 OFFSET2 BIN2 ...)
define write_flash
	esptool.py --chip $(CHIP) -p $(port) -b $(BAUDRATE) \
		--before=default_reset --after=hard_reset write_flash \
		--flash_mode $(FLASH_MODE) --flash_freq $(FLASH_FREQ) --flash_size $(FLASH_SIZE) \
		$(1)
endef

# Write flash with encryption
# Usage: $(call write_flash_encrypted,OFFSET1 BIN1 OFFSET2 BIN2 ...)
define write_flash_encrypted
	esptool.py --no-stub --chip $(CHIP) -p $(port) -b $(BAUDRATE) \
		--before=default_reset --after=hard_reset write_flash \
		--flash_mode $(FLASH_MODE) --flash_freq $(FLASH_FREQ) --flash_size $(FLASH_SIZE) \
		--encrypt $(1)
endef

# Write flash with encryption and --force flag
# Usage: $(call write_flash_encrypted_force,OFFSET1 BIN1 OFFSET2 BIN2 ...)
define write_flash_encrypted_force
	esptool.py --no-stub --chip $(CHIP) -p $(port) -b $(BAUDRATE) \
		--before=default_reset --after=hard_reset write_flash \
		--flash_mode $(FLASH_MODE) --flash_freq $(FLASH_FREQ) --flash_size $(FLASH_SIZE) \
		--encrypt --force $(1)
endef

# Write flash with --force flag (non-encrypted)
# Usage: $(call write_flash_force,OFFSET1 BIN1 OFFSET2 BIN2 ...)
define write_flash_force
	esptool.py --no-stub --chip $(CHIP) -p $(port) -b $(BAUDRATE) \
		--before=default_reset --after=hard_reset write_flash \
		--flash_mode $(FLASH_MODE) --flash_freq $(FLASH_FREQ) --flash_size $(FLASH_SIZE) \
		--force $(1)
endef

# Sign binary with secure boot v2
# Usage: $(call sign_binary,INPUT_BIN,OUTPUT_BIN)
define sign_binary
	espsecure.py sign_data --keyfile $(sign_key) --version 2 $(1) --output $(2)
endef

# Encrypt flash data
# Usage: $(call encrypt_flash_data,INPUT_BIN,OUTPUT_BIN,ADDRESS)
define encrypt_flash_data
	espsecure.py encrypt_flash_data --aes_xts --keyfile $(FLASH_ENCRYPT_KEY) \
		--address $(3) -o $(2) $(1)
endef
