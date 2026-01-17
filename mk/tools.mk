# tools.mk - Tool checking and validation
# Verifies required tools are available before build/flash operations

# ============================================================
# Individual Tool Checks
# ============================================================

.PHONY: check_idf
check_idf:
	@if ! command -v idf.py > /dev/null; then \
		echo "Error: idf.py not found. Source ESP-IDF's export.sh first."; \
		exit 1; \
	fi
	@if ! command -v esptool.py > /dev/null; then \
		echo "Error: esptool.py not found. Source ESP-IDF's export.sh first."; \
		exit 1; \
	fi

.PHONY: check_littlefs
check_littlefs:
	@if ! command -v littlefs-python > /dev/null; then \
		echo "Error: littlefs-python not found. Run: pip install littlefs-python"; \
		exit 1; \
	fi

.PHONY: check_jq
check_jq:
	@if ! command -v jq > /dev/null; then \
		echo "Error: jq not found. Install it using your package manager."; \
		exit 1; \
	fi

# ============================================================
# Combined Tool Check
# ============================================================

.PHONY: check_tools
check_tools: check_idf check_littlefs check_jq
