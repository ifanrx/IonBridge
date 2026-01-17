# Detect OS
ifeq ($(OS),Windows_NT)
	detected_OS := windows
else
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Darwin)
		detected_OS := macosx
	else
		detected_OS := $(shell uname | tr '[:upper:]' '[:lower:]')
	endif
endif

# Default values
VERSION ?= 0.4.9.1
OS ?= $(detected_OS)-x86

# Directory to store the downloaded and extracted files
MAKEFILE_DIR := $(patsubst %/,%,$(dir $(realpath $(firstword $(MAKEFILE_LIST)))))
PROJECT_DIR := $(MAKEFILE_DIR)/../..
OUT_DIR := $(MAKEFILE_DIR)/nanopb

# Check for system protoc first (for architecture compatibility)
SYSTEM_PROTOC := $(shell which protoc 2>/dev/null)

# URL to download Nanopb from
BASE_URL := https://jpa.kapsi.fi/nanopb/download
TAR_URL := $(BASE_URL)/nanopb-$(VERSION)-$(OS).tar.gz
ZIP_URL := $(BASE_URL)/nanopb-$(VERSION)-$(OS).zip

# Choose extractor command based on OS
ifeq ($(OS),windows-x86)
	EXTRACT_CMD := unzip -j -d $(OUT_DIR)
	FILE_URL := $(ZIP_URL)
else
	EXTRACT_CMD := tar -xz --strip-components=1 -C $(OUT_DIR)
	FILE_URL := $(TAR_URL)
endif

# Determine which protoc to use
ifneq ($(SYSTEM_PROTOC),)
	# Use system protoc with nanopb plugin when available (avoids architecture issues)
	USE_SYSTEM_PROTOC := 1
	GENERATOR_BIN := $(SYSTEM_PROTOC) --plugin=protoc-gen-nanopb=$(OUT_DIR)/generator/protoc-gen-nanopb
	GENERATOR_BIN_PY := $(SYSTEM_PROTOC)
	NANOPB_PROTO := $(OUT_DIR)/generator/proto/nanopb.proto
else
	# Fall back to downloaded protoc
	USE_SYSTEM_PROTOC := 0
	GENERATOR_BIN := $(OUT_DIR)/generator-bin/protoc
	GENERATOR_BIN_PY := $(GENERATOR_BIN)
	NANOPB_PROTO := $(OUT_DIR)/generator-bin/proto/nanopb.proto
endif

PROTO_DIR := $(MAKEFILE_DIR)/proto
PROTO_FILES := $(PROTO_DIR)/*.proto
OPTION_FILES := $(PROTO_DIR)/*.options
NANOPB_OPTS := --nanopb_opt=-I$(PROTO_DIR)

# Targets
.PHONY: all build clean download check-deps

all: check-deps download build build_py

check-deps:
ifeq ($(USE_SYSTEM_PROTOC),1)
	@echo "Using system protoc at $(SYSTEM_PROTOC)"
	@python3 -c "import google.protobuf; assert google.protobuf.__version__.startswith('5.28')" 2>/dev/null || (echo "Installing required Python packages (protobuf 5.28.1)..." && pip3 install protobuf==5.28.1 grpcio-tools)
else
	@echo "Using bundled protoc from nanopb package"
endif

build: $(PROTO_FILES) $(OPTION_FILES)
ifeq ($(USE_SYSTEM_PROTOC),1)
	$(GENERATOR_BIN) $(NANOPB_OPTS) --nanopb_out=. -I=$(PROTO_DIR) $(PROTO_FILES)
else
	$(GENERATOR_BIN) "$(NANOPB_OPTS)" --nanopb_out=. -I=$(PROTO_DIR) $(PROTO_FILES)
endif

build_py: $(PROTO_FILES)
ifeq ($(USE_SYSTEM_PROTOC),1)
	$(GENERATOR_BIN_PY) --pyi_out=$(PROJECT_DIR)/python/typings \
		--python_out=$(PROJECT_DIR)/python \
		-I=$(PROTO_DIR) \
		-I=$(OUT_DIR)/generator/proto \
		$(PROTO_FILES) $(NANOPB_PROTO)
else
	$(GENERATOR_BIN_PY) --pyi_out=$(PROJECT_DIR)/python/typings \
		--python_out=$(PROJECT_DIR)/python \
		-I=$(PROTO_DIR) \
		$(PROTO_FILES) $(NANOPB_PROTO)
endif

download: $(OUT_DIR)
	@echo "Nanopb downloaded and extracted to $(OUT_DIR)"

$(OUT_DIR):
	mkdir -p $(OUT_DIR)
	curl -L $(FILE_URL) | $(EXTRACT_CMD)

clean:
	rm -rf $(OUT_DIR)
