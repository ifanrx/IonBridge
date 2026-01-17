# Memory map for builds without coredump partition

# Partition table
PARTITION_TABLE_OFFSET := 0xc000

# LittleFS partition (larger without coredump)
LITTLEFS_OFFSET := 0x330000
LITTLEFS_LEN := 0xC5000

# Protected data partition
PROTECTED_DATA_OFFSET := 0x3FB000
PROTECTED_DATA_LEN := 0x5000

# PHY init data
PHY_INIT_OFFSET := 0x3FA000

# NVS data partition
NVS_DATA_OFFSET := 0x3F6000
NVS_DATA_LEN := 0x4000

# Secure NVS key
SECURE_NVS_KEY_OFFSET := 0x3f5000
