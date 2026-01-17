#!/usr/bin/env python3
"""
Convert images to raw bitmap format for GPU display.

Output format:
- 96x32 pixels, 1-bit monochrome (384 bytes)
- MSB-first, row-major order
- Each byte represents 8 horizontal pixels
- 1 = white pixel, 0 = black pixel

Usage:
    python image_to_raw_bitmap.py input.png output.bin
    python image_to_raw_bitmap.py --batch input_dir/ output_dir/
    python image_to_raw_bitmap.py --gif animation.gif output_dir/
    python image_to_raw_bitmap.py --video animation.mp4 output_dir/
    python image_to_raw_bitmap.py --video animation.mp4 output_dir/ --sample-fps 10
    python image_to_raw_bitmap.py --video animation.mp4 output_dir/ --max-frames 200
    python image_to_raw_bitmap.py --video animation.mp4 output_dir/ --center --max-frames 200
"""

import argparse
import os
import sys
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    print("Error: Pillow library required. Install with: pip install Pillow")
    sys.exit(1)

try:
    import cv2
except ImportError:
    cv2 = None  # Optional dependency for MP4/video support

# Default display dimensions (can be overridden with --width/--height)
DISPLAY_WIDTH = 96
DISPLAY_HEIGHT = 32
FRAMEBUFFER_SIZE = (DISPLAY_WIDTH * DISPLAY_HEIGHT) // 8  # 384 bytes


def get_pre_rotation_dimensions(rotate, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT):
    """
    Get image dimensions before rotation to achieve target output.

    Args:
        rotate: Rotation angle (0, 90, 180, 270)
        width: Target output width
        height: Target output height

    Returns:
        Tuple of (width, height)
    """
    if rotate in [90, 270]:
        return (height, width)  # Swap for portrait input
    else:
        return (width, height)  # Landscape input


def apply_rotation_transform(x, y, rotate, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT):
    """
    Apply rotation coordinate transformation.
    Maps framebuffer position (x, y) to source image coordinates.

    Args:
        x: X coordinate in framebuffer
        y: Y coordinate in framebuffer
        rotate: Rotation angle (0, 90, 180, 270)
        width: Target output width
        height: Target output height

    Returns:
        Tuple of (x_src, y_src) in source image
    """
    if rotate == 0:
        return x, y
    elif rotate == 90:
        # Source is height x width, rotated 90° CW
        return (height - 1) - y, x
    elif rotate == 180:
        # Source is width x height, rotated 180°
        return (width - 1) - x, (height - 1) - y
    elif rotate == 270:
        # Source is height x width, rotated 270° CW (90° CCW)
        return y, (width - 1) - x
    else:
        return x, y


def convert_image_to_raw_bitmap(
    input_path, output_path, threshold=128, invert=False, rotate=0, center=False,
    width=None, height=None, column_major=False
):
    """
    Convert image to raw bitmap format.

    Args:
        input_path: Path to input image file
        output_path: Path to output .bin file
        threshold: Grayscale threshold for 1-bit conversion (0-255)
        invert: If True, invert black/white
        rotate: Rotation angle in degrees (0, 90, 180, 270)
        center: If True, center content preserving aspect ratio
        width: Custom output width (None = use default 96)
        height: Custom output height (None = use default 32, must be multiple of 8)
        column_major: If True, output column-major format (for GPU partial uploads)
    """
    # Use custom or default dimensions
    out_width = width if width else DISPLAY_WIDTH
    out_height = height if height else DISPLAY_HEIGHT

    try:
        # Load image
        img = Image.open(input_path)
        print(f"Loaded {input_path}: {img.size}, mode={img.mode}")

        # Handle transparency with black background
        if img.mode in ("RGBA", "LA") or (img.mode == "P" and "transparency" in img.info):
            # Has transparency - convert to RGB with black background
            if img.mode == "P":
                img = img.convert("RGBA")
            background = Image.new("RGB", img.size, (0, 0, 0))
            if img.mode in ("RGBA", "LA"):
                background.paste(img, mask=img.split()[-1])  # Use alpha channel as mask
            img = background

        # Convert to grayscale
        img = img.convert("L")

        # Resize to pre-rotation dimensions with aspect ratio preservation
        target_size = get_pre_rotation_dimensions(rotate, out_width, out_height)
        target_width, target_height = target_size

        if img.size != target_size:
            # Calculate aspect ratios
            src_width, src_height = img.size
            src_aspect = src_width / src_height
            target_aspect = target_width / target_height

            print(f"Resizing from {img.size} to {target_size[0]}x{target_size[1]}")

            # Center if requested and aspect ratios differ
            if center and src_aspect != target_aspect:
                # Calculate dimensions to preserve aspect ratio
                if src_aspect > target_aspect:
                    # Source is wider - fit to width
                    new_width = target_width
                    new_height = int(src_height * (new_width / src_width))
                    y_offset = (target_height - new_height) // 2
                    x_offset = 0
                else:
                    # Source is taller - fit to height
                    new_height = target_height
                    new_width = int(src_width * (new_height / src_height))
                    x_offset = (target_width - new_width) // 2
                    y_offset = 0

                img = img.resize((new_width, new_height), Image.LANCZOS)

                # Create black canvas and paste centered
                canvas = Image.new("L", target_size, 0)
                canvas.paste(img, (x_offset, y_offset))
                img = canvas
                print(f"  Centered {new_width}x{new_height} content (offset={x_offset},{y_offset})")
            else:
                # Normal resize (stretch to fit)
                img = img.resize(target_size, Image.LANCZOS)

        # Convert to 1-bit using threshold
        img = img.point(lambda p: 255 if p > threshold else 0, mode="1")

        # Optionally invert
        if invert:
            img = Image.eval(img, lambda p: 255 - p)

        if column_major:
            # Column-major format (for GPU partial uploads)
            # Each column is height/8 bytes, bit 0 = row 0, bit 7 = row 7
            bytes_per_column = out_height // 8
            bitmap_size = out_width * bytes_per_column
            bitmap = bytearray(bitmap_size)

            for x in range(out_width):
                for y in range(out_height):
                    # Apply rotation transform
                    x_src, y_src = apply_rotation_transform(x, y, rotate, out_width, out_height)
                    pixel = img.getpixel((x_src, y_src))

                    # Column-major: byte = (col * bytes_per_column + row // 8), bit = (row % 8)
                    byte_index = x * bytes_per_column + y // 8
                    bit_index = y % 8

                    # Set bit if pixel is white
                    if pixel:
                        bitmap[byte_index] |= 1 << bit_index

            print(f"Output format: column-major, {out_width}x{out_height}, {bytes_per_column} bytes/column")
        else:
            # Row-major format (MSB-first continuous bit stream, for GPU framebuffer)
            # GPU expects: bit_idx = y * width + x, bit = 7 - (bit_idx % 8)
            bitmap_size = (out_width * out_height) // 8
            bitmap = bytearray(bitmap_size)

            for y in range(out_height):
                for x in range(out_width):
                    # Apply rotation transform
                    x_src, y_src = apply_rotation_transform(x, y, rotate, out_width, out_height)
                    pixel = img.getpixel((x_src, y_src))

                    # Calculate byte index and bit position (continuous bit stream)
                    bit_idx = y * out_width + x
                    byte_index = bit_idx // 8
                    bit_position = 7 - (bit_idx % 8)  # MSB first

                    # Set bit if pixel is white (255 in mode '1')
                    if pixel:
                        bitmap[byte_index] |= 1 << bit_position

            print(f"Output format: row-major MSB-first, {out_width}x{out_height}")

        # Write to file
        with open(output_path, "wb") as f:
            f.write(bitmap)

        print(f"Written {output_path} ({len(bitmap)} bytes)")
        return True

    except Exception as e:
        print(f"Error converting {input_path}: {e}")
        import traceback
        traceback.print_exc()
        return False


def batch_convert(input_dir, output_dir, threshold=128, invert=False, rotate=0, center=False):
    """
    Convert all images in a directory.

    Args:
        input_dir: Directory containing input images
        output_dir: Directory for output .bin files
        threshold: Grayscale threshold for 1-bit conversion
        invert: If True, invert black/white
        rotate: Rotation angle in degrees (0, 90, 180, 270)
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)

    if not input_path.is_dir():
        print(f"Error: {input_dir} is not a directory")
        return False

    # Create output directory if needed
    output_path.mkdir(parents=True, exist_ok=True)

    # Supported image formats
    extensions = {".png", ".jpg", ".jpeg", ".bmp", ".gif", ".tiff"}

    success_count = 0
    fail_count = 0

    for input_file in sorted(input_path.iterdir()):
        if input_file.suffix.lower() not in extensions:
            continue

        output_file = output_path / (input_file.stem + ".bin")

        if convert_image_to_raw_bitmap(
            input_file, output_file, threshold, invert, rotate, center
        ):
            success_count += 1
        else:
            fail_count += 1

    print(f"\nBatch conversion complete: {success_count} success, {fail_count} failed")
    return fail_count == 0


def extract_gif_frames(gif_path, output_dir, threshold=128, invert=False, rotate=0, center=False):
    """
    Extract frames from animated GIF and convert to raw bitmaps.

    Args:
        gif_path: Path to input GIF file
        output_dir: Directory for output .bin files
        threshold: Grayscale threshold for 1-bit conversion
        invert: If True, invert black/white
        rotate: Rotation angle in degrees (0, 90, 180, 270)

    Returns:
        Dictionary with frame_count, duration_ms per frame
    """
    try:
        gif = Image.open(gif_path)
        print(f"Loaded {gif_path}: {gif.size}, mode={gif.mode}")

        # Check if animated
        if not hasattr(gif, 'n_frames'):
            print("Not an animated GIF, converting as single image")
            frame_count = 1
        else:
            frame_count = gif.n_frames
            print(f"Animated GIF with {frame_count} frames")

        # Create output directory
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Extract frame durations (in milliseconds)
        durations = []
        success_count = 0

        for frame_idx in range(frame_count):
            gif.seek(frame_idx)

            # Get frame duration (default 100ms if not specified)
            duration = gif.info.get('duration', 100)
            durations.append(duration)

            # Copy the current frame (important for animated GIFs)
            # This ensures we get the complete frame, not just deltas
            frame = gif.copy()

            # Convert to RGB, handling palette and transparency
            # Use BLACK background (0,0,0) since display expects black=0 (unlit)
            if frame.mode == "P":
                # Palette mode - check for transparency
                if "transparency" in frame.info:
                    frame = frame.convert("RGBA")
                    background = Image.new("RGB", frame.size, (0, 0, 0))
                    background.paste(frame, mask=frame.split()[3])
                    frame = background
                else:
                    frame = frame.convert("RGB")
            elif frame.mode == "RGBA":
                # Has alpha channel
                background = Image.new("RGB", frame.size, (0, 0, 0))
                background.paste(frame, mask=frame.split()[3])
                frame = background
            else:
                # Already RGB or L
                frame = frame.convert("RGB")

            # Convert to grayscale
            frame = frame.convert("L")

            # Resize to pre-rotation dimensions
            target_size = get_pre_rotation_dimensions(rotate)
            target_width, target_height = target_size

            if frame.size != target_size:
                # Calculate aspect ratios
                src_width, src_height = frame.size
                src_aspect = src_width / src_height
                target_aspect = target_width / target_height

                # Center if requested and aspect ratios differ
                if center and src_aspect != target_aspect:
                    # Calculate dimensions to preserve aspect ratio
                    if src_aspect > target_aspect:
                        # Source is wider - fit to width
                        new_width = target_width
                        new_height = int(src_height * (new_width / src_width))
                        y_offset = (target_height - new_height) // 2
                        x_offset = 0
                    else:
                        # Source is taller - fit to height
                        new_height = target_height
                        new_width = int(src_width * (new_height / src_height))
                        x_offset = (target_width - new_width) // 2
                        y_offset = 0

                    frame = frame.resize((new_width, new_height), Image.LANCZOS)

                    # Create black canvas and paste centered
                    canvas = Image.new("L", target_size, 0)
                    canvas.paste(frame, (x_offset, y_offset))
                    frame = canvas
                else:
                    # Normal resize (stretch to fit)
                    frame = frame.resize(target_size, Image.LANCZOS)

            # Convert to 1-bit
            frame = frame.point(lambda p: 255 if p > threshold else 0, mode="1")

            # Optionally invert
            if invert:
                frame = Image.eval(frame, lambda p: 255 - p)

            # Convert to raw bitmap
            bitmap = bytearray(FRAMEBUFFER_SIZE)

            for y in range(DISPLAY_HEIGHT):
                for x in range(DISPLAY_WIDTH):
                    # Apply rotation transform
                    x_src, y_src = apply_rotation_transform(x, y, rotate)
                    pixel = frame.getpixel((x_src, y_src))
                    byte_index = (y * DISPLAY_WIDTH + x) // 8
                    bit_position = 7 - (x % 8)
                    if pixel:
                        bitmap[byte_index] |= 1 << bit_position

            # Write frame
            output_file = output_path / f"frame_{frame_idx}.bin"
            with open(output_file, "wb") as f:
                f.write(bitmap)

            print(f"  Frame {frame_idx}: {output_file} ({duration}ms)")
            success_count += 1

        # Write metadata file
        metadata_file = output_path / "animation.txt"
        with open(metadata_file, "w") as f:
            f.write(f"# Animation metadata for {Path(gif_path).name}\n")
            f.write(f"frame_count={frame_count}\n")
            f.write(f"total_duration_ms={sum(durations)}\n")
            f.write(f"avg_duration_ms={sum(durations) // frame_count}\n")
            f.write(f"\n# Per-frame durations (ms)\n")
            for idx, dur in enumerate(durations):
                f.write(f"frame_{idx}={dur}\n")

        print(f"\nExtracted {success_count} frames from {gif_path}")
        print(f"Metadata saved to {metadata_file}")

        return {
            "frame_count": frame_count,
            "durations": durations,
            "avg_duration_ms": sum(durations) // frame_count if frame_count > 0 else 0,
        }

    except Exception as e:
        print(f"Error extracting GIF frames: {e}")
        import traceback
        traceback.print_exc()
        return None


def extract_video_frames(video_path, output_dir, threshold=128, invert=False, rotate=0, target_fps=None, max_frames=None, center=False):
    """
    Extract frames from MP4/video file and convert to raw bitmaps.

    Args:
        video_path: Path to input video file (MP4, AVI, MOV, etc.)
        output_dir: Directory for output .bin files
        threshold: Grayscale threshold for 1-bit conversion
        invert: If True, invert black/white
        rotate: Rotation angle in degrees (0, 90, 180, 270)
        target_fps: Target FPS for frame sampling (None = extract all frames)
        max_frames: Maximum number of frames to extract (None = no limit, overrides target_fps)

    Returns:
        Dictionary with frame_count, fps, durations per frame
    """
    if cv2 is None:
        print("Error: opencv-python required for video support. Install with: pip install opencv-python")
        return None

    try:
        video = cv2.VideoCapture(str(video_path))

        if not video.isOpened():
            print(f"Error: Cannot open video file {video_path}")
            return None

        # Get video properties
        fps = video.get(cv2.CAP_PROP_FPS)
        frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

        print(f"Loaded {video_path}: {frame_count} frames @ {fps:.2f} FPS")

        # Calculate frame sampling
        use_frame_limit = False
        if max_frames and max_frames < frame_count:
            # Use max_frames limit (simple frame skipping)
            use_frame_limit = True
            frame_step = frame_count / max_frames
            effective_fps = max_frames / (frame_count / fps)
            print(f"Limiting to {max_frames} frames (extracting every {frame_step:.2f}th frame)")
        elif target_fps and target_fps < fps:
            # Use FPS-based sampling
            effective_fps = target_fps
            frame_interval = 1.0 / target_fps
            expected_frame_count = int(frame_count * target_fps / fps)
            print(f"Sampling at {target_fps} FPS (extracting ~{expected_frame_count} of {frame_count} frames)")
        else:
            # Extract all frames
            effective_fps = fps
            frame_interval = 1.0 / fps
            print(f"Extracting all frames at {fps:.2f} FPS")

        # Create output directory
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Calculate frame duration in milliseconds
        frame_duration_ms = int(1000 / effective_fps) if effective_fps > 0 else 33
        durations = []
        success_count = 0

        output_frame_idx = 0
        source_frame_idx = 0
        next_output_time = 0.0  # Time when we should output next frame
        next_frame_index = 0.0  # For max_frames mode

        while True:
            ret, frame = video.read()
            if not ret:
                break

            # Check if we should extract this frame
            should_extract = False

            if use_frame_limit:
                # max_frames mode: extract when source_frame_idx >= next_frame_index
                if source_frame_idx >= int(next_frame_index):
                    should_extract = True
                    next_frame_index += frame_step
                    if max_frames and output_frame_idx >= max_frames:
                        break  # Stop if we've reached the limit
            elif target_fps and target_fps < fps:
                # FPS-based sampling
                current_time = source_frame_idx / fps
                if current_time >= next_output_time - 0.001:
                    should_extract = True
                    next_output_time += frame_interval
            else:
                # Extract all frames
                should_extract = True

            if not should_extract:
                source_frame_idx += 1
                continue

            # OpenCV reads as BGR, convert to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Convert numpy array to PIL Image
            pil_frame = Image.fromarray(frame_rgb)

            # Convert to grayscale
            pil_frame = pil_frame.convert("L")

            # Resize with optional centering
            target_size = get_pre_rotation_dimensions(rotate)
            target_width, target_height = target_size

            if pil_frame.size != target_size:
                # Calculate aspect ratios
                src_width, src_height = pil_frame.size
                src_aspect = src_width / src_height
                target_aspect = target_width / target_height

                # Print resize info only for first frame
                if output_frame_idx == 0:
                    print(f"  Resizing from {src_width}x{src_height} to {target_width}x{target_height}")

                # Center if requested and aspect ratios differ
                if center and src_aspect != target_aspect:
                    # Calculate dimensions to preserve aspect ratio
                    if src_aspect > target_aspect:
                        # Source is wider - fit to width
                        new_width = target_width
                        new_height = int(src_height * (new_width / src_width))
                        y_offset = (target_height - new_height) // 2
                        x_offset = 0
                    else:
                        # Source is taller - fit to height
                        new_height = target_height
                        new_width = int(src_width * (new_height / src_height))
                        x_offset = (target_width - new_width) // 2
                        y_offset = 0

                    pil_frame = pil_frame.resize((new_width, new_height), Image.LANCZOS)

                    # Create black canvas and paste centered
                    canvas = Image.new("L", target_size, 0)
                    canvas.paste(pil_frame, (x_offset, y_offset))
                    pil_frame = canvas

                    if output_frame_idx == 0:
                        print(f"  Centered {new_width}x{new_height} content (offset={x_offset},{y_offset})")
                else:
                    # Normal resize (stretch to fit)
                    pil_frame = pil_frame.resize(target_size, Image.LANCZOS)

            # Convert to 1-bit
            pil_frame = pil_frame.point(lambda p: 255 if p > threshold else 0, mode="1")

            # Optionally invert
            if invert:
                pil_frame = Image.eval(pil_frame, lambda p: 255 - p)

            # Convert to raw bitmap
            bitmap = bytearray(FRAMEBUFFER_SIZE)

            for y in range(DISPLAY_HEIGHT):
                for x in range(DISPLAY_WIDTH):
                    # Apply rotation transform
                    x_src, y_src = apply_rotation_transform(x, y, rotate)
                    pixel = pil_frame.getpixel((x_src, y_src))
                    byte_index = (y * DISPLAY_WIDTH + x) // 8
                    bit_position = 7 - (x % 8)
                    if pixel:
                        bitmap[byte_index] |= 1 << bit_position

            # Write frame
            output_file = output_path / f"frame_{output_frame_idx}.bin"
            with open(output_file, "wb") as f:
                f.write(bitmap)

            durations.append(frame_duration_ms)
            print(f"  Frame {output_frame_idx}: {output_file} ({frame_duration_ms}ms)")
            success_count += 1
            output_frame_idx += 1
            source_frame_idx += 1

        video.release()

        # Write metadata file
        metadata_file = output_path / "animation.txt"
        with open(metadata_file, "w") as f:
            f.write(f"# Animation metadata for {Path(video_path).name}\n")
            f.write(f"frame_count={output_frame_idx}\n")
            f.write(f"source_fps={fps:.2f}\n")
            f.write(f"source_frame_count={frame_count}\n")
            f.write(f"output_fps={effective_fps:.2f}\n")
            if use_frame_limit:
                f.write(f"max_frames_limit={max_frames}\n")
                f.write(f"frame_step={frame_step:.2f}\n")
            f.write(f"total_duration_ms={sum(durations)}\n")
            f.write(f"avg_duration_ms={frame_duration_ms}\n")
            f.write(f"\n# Per-frame durations (ms)\n")
            for idx, dur in enumerate(durations):
                f.write(f"frame_{idx}={dur}\n")

        print(f"\nExtracted {success_count} frames from {video_path}")
        print(f"Metadata saved to {metadata_file}")

        return {
            "frame_count": output_frame_idx,
            "fps": effective_fps,
            "durations": durations,
            "avg_duration_ms": frame_duration_ms,
        }

    except Exception as e:
        print(f"Error extracting video frames: {e}")
        import traceback
        traceback.print_exc()
        return None


def preview_bitmap(bitmap_path):
    """
    Preview a raw bitmap file as ASCII art.

    Args:
        bitmap_path: Path to .bin file
    """
    try:
        with open(bitmap_path, "rb") as f:
            bitmap = f.read()

        if len(bitmap) != FRAMEBUFFER_SIZE:
            print(f"Error: Invalid bitmap size ({len(bitmap)} bytes, expected {FRAMEBUFFER_SIZE})")
            return

        print(f"Preview of {bitmap_path}:")
        print("+" + "-" * DISPLAY_WIDTH + "+")

        for y in range(DISPLAY_HEIGHT):
            line = "|"
            for x in range(DISPLAY_WIDTH):
                byte_index = (y * DISPLAY_WIDTH + x) // 8
                bit_position = 7 - (x % 8)
                pixel = (bitmap[byte_index] >> bit_position) & 1
                line += "█" if pixel else " "
            line += "|"
            print(line)

        print("+" + "-" * DISPLAY_WIDTH + "+")

    except Exception as e:
        print(f"Error previewing {bitmap_path}: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert images to raw bitmap for GPU display (96x32, 1-bit)"
    )

    parser.add_argument("input", help="Input image file or directory")
    parser.add_argument("output", help="Output .bin file or directory", nargs="?")

    parser.add_argument(
        "--batch",
        action="store_true",
        help="Batch mode: convert all images in input directory",
    )
    parser.add_argument(
        "--gif",
        action="store_true",
        help="GIF mode: extract all frames from animated GIF",
    )
    parser.add_argument(
        "--video",
        action="store_true",
        help="Video mode: extract all frames from MP4/AVI/MOV video",
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=128,
        help="Grayscale threshold for 1-bit conversion (0-255, default: 128)",
    )
    parser.add_argument(
        "--invert", action="store_true", help="Invert black and white"
    )
    parser.add_argument(
        "--center",
        action="store_true",
        help="Preserve aspect ratio and center content (default: stretch to fill)",
    )
    parser.add_argument(
        "--rotate",
        type=int,
        choices=[0, 90, 180, 270],
        default=0,
        help="Rotate image clockwise (degrees, default: 0)",
    )
    parser.add_argument(
        "--sample-fps",
        type=float,
        default=None,
        help="Target FPS for video sampling (reduces frame count, default: extract all frames)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="Maximum number of frames to extract from video (overrides --sample-fps)",
    )
    parser.add_argument(
        "--preview", action="store_true", help="Preview bitmap file as ASCII art"
    )
    parser.add_argument(
        "--width",
        type=int,
        default=None,
        help="Custom output width (default: 96, for single image mode only)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=None,
        help="Custom output height (default: 32, must be multiple of 8, for single image mode only)",
    )
    parser.add_argument(
        "--column-major",
        action="store_true",
        help="Output column-major format (for GPU partial uploads, use with --width/--height)",
    )

    args = parser.parse_args()

    # Preview mode
    if args.preview:
        preview_bitmap(args.input)
        return

    # Validate arguments
    if not args.output:
        print("Error: output argument required")
        parser.print_help()
        sys.exit(1)

    # Video, GIF, batch, or single conversion
    if args.video:
        result = extract_video_frames(
            args.input, args.output, args.threshold, args.invert, args.rotate, args.sample_fps, args.max_frames, args.center
        )
        success = result is not None
    elif args.gif:
        result = extract_gif_frames(
            args.input, args.output, args.threshold, args.invert, args.rotate, args.center
        )
        success = result is not None
    elif args.batch:
        success = batch_convert(
            args.input, args.output, args.threshold, args.invert, args.rotate, args.center
        )
    else:
        success = convert_image_to_raw_bitmap(
            args.input, args.output, args.threshold, args.invert, args.rotate, args.center,
            args.width, args.height, args.column_major
        )

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
