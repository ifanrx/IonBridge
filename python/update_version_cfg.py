#!/usr/bin/env python3
# -*- coding=utf-8 -*-

import gzip
import hashlib
import json
import struct
import subprocess
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import asyncclick as click

# Embedded checksum location in SW3566 firmware
CHECKSUM_ADDRESS: int = 0x1DFA0
CHECKSUM_LENGTH: int = 20


def git_revision_hash() -> str:
    return subprocess.check_output(["git", "rev-parse", "HEAD"]).decode("ascii").strip()


def sw3566_embedded_checksum(path: Path) -> str:
    """Extract embedded checksum from offset 0x1dfa0 in uncompressed firmware.
    Used to compare with running chip's reported checksum."""
    return (
        subprocess.check_output(
            [
                "xxd",
                "-seek",
                str(CHECKSUM_ADDRESS),
                "-l",
                str(CHECKSUM_LENGTH),
                "-p",
                str(path),
            ]
        )
        .decode("ascii")
        .strip()
    )


def sw3566_file_sha1(path: Path) -> str:
    """Compute SHA-1 hash of the file (compressed or uncompressed).
    Used to verify file integrity on littlefs before upgrade."""
    return hashlib.sha1(path.read_bytes()).hexdigest()


def compress_file_gzip_deterministic(source_path: Path, dest_path: Path) -> bool:
    """Compress file using Python gzip for deterministic output.

    Uses mtime=0 and consistent compression level to guarantee
    byte-for-byte identical output across all environments.
    This solves the issue where different gzip implementations
    produce different compressed bytes from the same input.
    """
    try:
        source_data = source_path.read_bytes()
        with open(dest_path, 'wb') as f_out:
            with gzip.GzipFile(
                filename='',
                mode='wb',
                compresslevel=6,
                mtime=0,
                fileobj=f_out
            ) as gz:
                gz.write(source_data)
        return True
    except Exception as e:
        print(f"Compression failed: {e}")
        return False


def read_xz_vli(f) -> int:
    """Read XZ Variable-Length Integer"""
    value = 0
    shift = 0

    while True:
        byte_data = f.read(1)
        if not byte_data:
            return 0
        byte = byte_data[0]

        value |= (byte & 0x7F) << shift
        shift += 7

        if shift > 63:  # Overflow protection
            return 0

        if not (byte & 0x80):  # Continuation bit not set
            break

    return value


def get_xz_uncompressed_size(xz_path: Path) -> int:
    """Parse XZ file to extract uncompressed size from Index"""
    try:
        with open(xz_path, 'rb') as f:
            # Seek to Stream Footer (last 12 bytes)
            f.seek(-12, 2)
            footer = f.read(12)

            # Verify footer magic bytes (0x59, 0x5A = "YZ")
            if footer[10] != 0x59 or footer[11] != 0x5A:
                return 0

            # Extract Backward Size (4 bytes at offset 4, little-endian)
            backward_size = struct.unpack('<I', footer[4:8])[0]

            # Backward Size is stored as (actual_size / 4) - 1
            index_size = (backward_size + 1) * 4

            # Seek to start of Index
            f.seek(-(12 + index_size), 2)

            # Read Index Indicator (should be 0x00)
            index_indicator = f.read(1)[0]
            if index_indicator != 0x00:
                return 0

            # Read Number of Records (VLI)
            num_records = read_xz_vli(f)
            if num_records == 0:
                return 0

            # Sum uncompressed sizes from all Block records
            total_uncompressed = 0
            for _ in range(num_records):
                unpadded_size = read_xz_vli(f)  # Don't need this
                if unpadded_size == 0:
                    return 0

                uncompressed_size = read_xz_vli(f)  # This is what we want
                if uncompressed_size == 0:
                    return 0

                total_uncompressed += uncompressed_size

            return total_uncompressed
    except Exception as e:
        print(f"Error parsing XZ file {xz_path}: {e}")
        return 0


def get_gzip_uncompressed_size(gz_path: Path) -> int:
    """Extract uncompressed size from GZIP footer (last 4 bytes)"""
    try:
        with open(gz_path, 'rb') as f:
            f.seek(-4, 2)  # Last 4 bytes
            size_bytes = f.read(4)
            return struct.unpack('<I', size_bytes)[0]
    except Exception as e:
        print(f"Error parsing GZIP file {gz_path}: {e}")
        return 0


def detect_compression_type() -> str:
    """Detect compression type from environment variables, fallback to sdkconfig"""
    import os

    # Try environment variables first (allows override)
    if os.environ.get("USE_XZ_COMPRESSION") == "1":
        return "xz"
    elif os.environ.get("USE_GZIP_COMPRESSION") == "1":
        return "gzip"

    # Fallback to sdkconfig
    sdkconfig_path = Path("sdkconfig")
    if sdkconfig_path.exists():
        config_text = sdkconfig_path.read_text()
        if "CONFIG_STORAGE_COMPRESSION_XZ=y" in config_text:
            return "xz"
        elif "CONFIG_STORAGE_COMPRESSION_GZIP=y" in config_text:
            return "gzip"

    return "none"


def get_firmware_size(base_path: Path, compression_type: str) -> Optional[int]:
    """Get firmware size - either from compressed file or uncompressed"""
    if compression_type == "xz":
        xz_path = Path(str(base_path) + ".xz")
        if xz_path.exists():
            size = get_xz_uncompressed_size(xz_path)
            if size > 0:
                compressed_size = xz_path.stat().st_size
                ratio = (1 - compressed_size / size) * 100
                print(f"XZ file {xz_path.name}: {compressed_size} -> {size} bytes ({ratio:.1f}% saved)")
                return size
    elif compression_type == "gzip":
        gz_path = Path(str(base_path) + ".gz")
        if gz_path.exists():
            size = get_gzip_uncompressed_size(gz_path)
            if size > 0:
                compressed_size = gz_path.stat().st_size
                ratio = (1 - compressed_size / size) * 100
                print(f"GZIP file {gz_path.name}: {compressed_size} -> {size} bytes ({ratio:.1f}% saved)")
                return size

    # Fallback to uncompressed file
    if base_path.exists():
        size = base_path.stat().st_size
        print(f"Uncompressed file {base_path.name}: size = {size} bytes")
        return size

    return None


@dataclass
class VersionConfig:
    CONFIG_APP_PROJECT_VER: str
    CONFIG_SW3566_FIRMWARE_NUMERIC_VERSION: int
    CONFIG_SW3566_FIRMWARE_SEMVER: str
    CONFIG_SW3566_EMBEDDED_CHECKSUM: str  # Embedded checksum for chip comparison
    CONFIG_SW3566_FILE_SHA1: str  # SHA-1 of compressed file for integrity
    CONFIG_SW3566_FIRMWARE_SIZE: int
    CONFIG_FPGA_FIRMWARE_NUMERIC_VERSION: int
    CONFIG_FPGA_FIRMWARE_SEMVER: str
    CONFIG_FPGA_FIRMWARE_SIZE: int
    CONFIG_ZRLIB_SEMVER: str

    # Store original config and product family for dump_to_json
    _original_cfg: dict = None
    _product_family: str = None

    @classmethod
    def from_json_file(cls, json_path):
        import os
        cfg = json.loads(Path(json_path).read_text())

        # Get PRODUCT_FAMILY from environment
        product_family = os.environ.get('PRODUCT_FAMILY', 'CP02')
        print(f"Using product family: {product_family}")

        # Get firmware configs: SW3566[product_family], FPGA[product_family]
        sw3566_cfg = cfg.get('SW3566', {}).get(product_family, {})
        fpga_cfg = cfg.get('FPGA', {}).get(product_family, {})

        if not sw3566_cfg:
            print(f"Note: SW3566.{product_family} not found, will be created")
        if not fpga_cfg:
            print(f"Note: FPGA.{product_family} not found, will be created")

        # Detect compression type
        compression_type = detect_compression_type()
        print(f"Detected compression type: {compression_type}")

        sw3566_version = sw3566_cfg.get('version', '')

        # 1. Get embedded checksum from SOURCE (uncompressed) firmware
        #    Used to compare with running chip's reported checksum
        sw3566_source_path = Path(Path.cwd() / "files" / product_family / "littlefs" / f"SW3566_{sw3566_version}.bin")
        if sw3566_source_path.exists():
            embedded_checksum = sw3566_embedded_checksum(sw3566_source_path)
            print(f"Embedded checksum from source: {sw3566_source_path}")
        else:
            # Use existing checksum from version.json if source file not found
            embedded_checksum = sw3566_cfg.get('checksum', 'N/A')
            print(f"Warning: SW3566 source file not found at {sw3566_source_path}, using existing checksum")

        # 2. Get SHA-1 of compressed file
        #    Used to verify file integrity on device before upgrade
        #    Try littlefs/ working dir first, then source dir (files/{product_family}/littlefs/)
        sw3566_littlefs_base = Path(Path.cwd() / "littlefs" / f"SW3566_{sw3566_version}.bin")
        sw3566_source_base = Path(Path.cwd() / "files" / product_family / "littlefs" / f"SW3566_{sw3566_version}.bin")

        if compression_type == "xz":
            ext = ".xz"
        elif compression_type == "gzip":
            ext = ".gz"
        else:
            ext = ""

        # Check littlefs/ first (prepared working dir), then source dir
        sw3566_compressed_path = None
        for base in [sw3566_littlefs_base, sw3566_source_base]:
            candidate = Path(str(base) + ext) if ext else base
            if candidate.exists():
                sw3566_compressed_path = candidate
                break

        if sw3566_compressed_path:
            file_sha1 = sw3566_file_sha1(sw3566_compressed_path)
            print(f"File SHA-1 from: {sw3566_compressed_path}")
        else:
            # Use existing file_sha1 from version.json if file not found
            file_sha1 = sw3566_cfg.get('file_sha1', '')
            print(f"Warning: SW3566 compressed file not found")

        # Get firmware sizes - try littlefs/ first, then source dir
        sw3566_size = get_firmware_size(sw3566_littlefs_base, compression_type)
        if sw3566_size is None:
            sw3566_size = get_firmware_size(sw3566_source_base, compression_type)
        if sw3566_size is None:
            sw3566_size = sw3566_cfg.get('size', 0)
            if sw3566_size > 0:
                print(f"Using SW3566 size from version.json: {sw3566_size} bytes")
            else:
                print(f"Warning: Could not determine SW3566 firmware size, using 0")

        fpga_version = fpga_cfg.get('version', '')
        fpga_littlefs_base = Path(Path.cwd() / "littlefs" / f"FPGA_{fpga_version}.bin")
        fpga_source_base = Path(Path.cwd() / "files" / product_family / "littlefs" / f"FPGA_{fpga_version}.bin")
        fpga_size = get_firmware_size(fpga_littlefs_base, compression_type)
        if fpga_size is None:
            fpga_size = get_firmware_size(fpga_source_base, compression_type)
        if fpga_size is None:
            fpga_size = fpga_cfg.get('size', 0)
            if fpga_size > 0:
                print(f"Using FPGA size from version.json: {fpga_size} bytes")
            else:
                print(f"Warning: Could not determine FPGA firmware size, using 0")

        instance = cls(
            CONFIG_APP_PROJECT_VER=cfg["version"],
            CONFIG_SW3566_FIRMWARE_NUMERIC_VERSION=sw3566_cfg.get("numeric", 0),
            CONFIG_SW3566_FIRMWARE_SEMVER=sw3566_version,
            CONFIG_SW3566_EMBEDDED_CHECKSUM=embedded_checksum,
            CONFIG_SW3566_FILE_SHA1=file_sha1,
            CONFIG_SW3566_FIRMWARE_SIZE=sw3566_size,
            CONFIG_FPGA_FIRMWARE_NUMERIC_VERSION=fpga_cfg.get("numeric", 0),
            CONFIG_FPGA_FIRMWARE_SEMVER=fpga_version,
            CONFIG_FPGA_FIRMWARE_SIZE=fpga_size,
            CONFIG_ZRLIB_SEMVER=cfg["zrlib"]["version"],
        )
        # Store for dump_to_json
        instance._original_cfg = cfg
        instance._product_family = product_family
        return instance

    def dump_to_json(self, json_path) -> None:
        cfg = self._original_cfg or {}
        product_family = self._product_family or 'CP02'

        # Update common fields
        cfg["version"] = self.CONFIG_APP_PROJECT_VER
        cfg["release_date"] = int(datetime.now().strftime("%Y%m%d%H"))
        cfg["git_commit"] = git_revision_hash()
        cfg["zrlib"] = {"version": self.CONFIG_ZRLIB_SEMVER}

        # Ensure new structure exists
        if 'SW3566' not in cfg:
            cfg['SW3566'] = {}
        if 'FPGA' not in cfg:
            cfg['FPGA'] = {}

        # Update the specific product family
        cfg['SW3566'][product_family] = {
            "numeric": self.CONFIG_SW3566_FIRMWARE_NUMERIC_VERSION,
            "version": self.CONFIG_SW3566_FIRMWARE_SEMVER,
            "checksum": self.CONFIG_SW3566_EMBEDDED_CHECKSUM,
            "file_sha1": self.CONFIG_SW3566_FILE_SHA1,
            "size": self.CONFIG_SW3566_FIRMWARE_SIZE,
        }
        cfg['FPGA'][product_family] = {
            "numeric": self.CONFIG_FPGA_FIRMWARE_NUMERIC_VERSION,
            "version": self.CONFIG_FPGA_FIRMWARE_SEMVER,
            "size": self.CONFIG_FPGA_FIRMWARE_SIZE,
        }

        Path(json_path).write_text(json.dumps(cfg, indent=2))


def update_version_cfg(version_cfg_path, new_version_dict):
    with open(version_cfg_path, "r") as f:
        lines = f.readlines()

    # Filter out internal fields that shouldn't go to sdkconfig
    filtered_dict = {k: v for k, v in new_version_dict.items() if not k.startswith('_')}

    # Track which keys were found and updated
    found_keys = set()

    for i, line in enumerate(lines):
        for key, value in filtered_dict.items():
            if not line.startswith(f"{key}="):
                continue
            found_keys.add(key)
            if type(value) is str:
                lines[i] = f'{key}="{value}"\n'
            else:
                lines[i] = f"{key}={value}\n"

    # Add missing keys at the end
    for key, value in filtered_dict.items():
        if key not in found_keys:
            if type(value) is str:
                lines.append(f'{key}="{value}"\n')
            else:
                lines.append(f"{key}={value}\n")

    with open(version_cfg_path, "w") as f:
        f.writelines(lines)


@click.group(invoke_without_command=True)
@click.pass_context
async def cli(ctx):
    """Version configuration and compression utilities."""
    if ctx.invoked_subcommand is None:
        # Default behavior: run update command with defaults
        await ctx.invoke(update)


@cli.command()
@click.argument("cfg_dir", type=click.Path(exists=True, dir_okay=True), default=".")
@click.argument(
    "version_cfg_json",
    type=click.Path(exists=True, dir_okay=False, file_okay=True),
    default="./version.json",
)
async def update(cfg_dir, version_cfg_json):
    """Update version configuration from version.json."""
    version_cfg = VersionConfig.from_json_file(version_cfg_json)
    update_version_cfg(
        "sdkconfig.defaults",
        asdict(version_cfg),
    )
    version_cfg.dump_to_json(version_cfg_json)


@cli.command()
@click.argument("source", type=click.Path(exists=True))
@click.argument("dest", type=click.Path())
async def compress(source, dest):
    """Compress a file with deterministic gzip.

    Uses Python's gzip module with mtime=0 to guarantee
    byte-for-byte identical output across all environments.
    """
    source_path = Path(source)
    dest_path = Path(dest)
    if compress_file_gzip_deterministic(source_path, dest_path):
        print(f"  Compressing {source_path.name} -> {dest_path.name}")
    else:
        raise click.ClickException(f"Compression failed: {source}")


if __name__ == "__main__":
    cli()
