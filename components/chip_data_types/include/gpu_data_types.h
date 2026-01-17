#ifndef GPU_DATA_TYPES_H_
#define GPU_DATA_TYPES_H_

#include <cstdint>

#ifndef __PACKED
#define __PACKED __attribute__((packed))
#endif

// PreComposite mode: applied to back buffer before drawing
typedef enum : uint8_t {
  PRE_COMPOSITE_MODE_KEEP = 0,   // Keep the current framebuffer
  PRE_COMPOSITE_MODE_CLEAR = 1,  // Clear the framebuffer
} PreCompositeMode;

// PostComposite mode: applied after drawing operations
typedef enum : uint8_t {
  POST_COMPOSITE_MODE_WAIT = 0,    // Wait, don't present yet
  POST_COMPOSITE_MODE_COMMIT = 1,  // Commit and signal OLED to present
} PostCompositeMode;

// BitBLT mode: pixel blending operations
typedef enum : uint8_t {
  BITBLT_MODE_COPY = 0,  // Overwrite destination
  BITBLT_MODE_OR = 1,    // Logical OR
  BITBLT_MODE_AND = 2,   // Logical AND
  BITBLT_MODE_XOR = 3,   // Logical XOR
} BitBLTMode;

// GPU animation modes
typedef enum : uint8_t {
  GPU_ANIMATION_OFF = 0,

  GPU_ANIMATION_FRAMEBUFFER = 1,  // User-controlled framebuffer

  GPU_ANIMATION_TEST_PATTERN = 2,

  GPU_ANIMATION_VERTICAL_POWER_VUMETER = 3,
  GPU_ANIMATION_HORIZONTAL_POWER_VUMETER = 4,
  GPU_ANIMATION_VERTICAL_POWER_VUMETER_PER_PORT = 5,
  GPU_ANIMATION_HORIZONTAL_POWER_VUMETER_PER_PORT = 6,

  GPU_ANIMATION_GAME_OF_LIFE = 7,

  GPU_ANIMATION_LAST = 0xFF,
} GPUAnimationIndex;

// Test pattern modes
typedef enum : uint8_t {
  GPU_TEST_PATTERN_OFF = 0,
  GPU_TEST_PATTERN_ON = 1,
  GPU_TEST_PATTERN_BLINK = 2,
} GPUTestPatternIndex;

// Font size selection
typedef enum : uint8_t {
  GPU_FONT_SIZE_SMALL = 0,  // Small font (e.g., 6x12)
  GPU_FONT_SIZE_LARGE = 1,  // Large font (e.g., 8x16)
} GPUFontSizeIndex;

// Text rotation (controls both glyph orientation and text flow direction)
typedef enum : uint8_t {
  GPU_TEXT_ROTATION_0 = 0,    // No rotation, text flows left->right
  GPU_TEXT_ROTATION_90 = 1,   // 90° clockwise, text flows top->bottom
  GPU_TEXT_ROTATION_180 = 2,  // 180°, text flows right->left
  GPU_TEXT_ROTATION_270 = 3,  // 270° clockwise, text flows bottom->top
} GPUTextRotation;

// Game of Life density levels (based on power)
typedef enum : uint8_t {
  GOL_DENSITY_LOW = 0,     // Low power [0, 10W)
  GOL_DENSITY_MEDIUM = 1,  // Medium power [10, 50W)
  GOL_DENSITY_HIGH = 2,    // High power [50W+]
} GOLDensityLevel;

// RPC Command IDs
typedef enum : uint16_t {
  CMD_SET_BACKLIGHT_INTENSITY = 0x0001,
  CMD_SET_ANIMATION_INDEX = 0x0002,
  CMD_SET_ANIMATION_DATA = 0x0003,
  CMD_DRAW_TEXT = 0x0004,
  CMD_PREPARE_FRAMEBUFFER = 0x0005,
  CMD_SET_FRAMEBUFFER = 0x0006,
  CMD_SET_DISPLAY_STATUS = 0x0007,
} RPCCommandID;

// RPC Response structure
typedef struct {
  uint8_t status;  // 0x00=OK, 0x01=ERROR, 0x02=BUSY
  uint8_t data;    // Command-specific return value
} __PACKED RPCResponse;

// Status codes
#define RPC_STATUS_OK 0x00
#define RPC_STATUS_ERROR 0x01
#define RPC_STATUS_BUSY 0x02

// Framebuffer chunk protocol constants
#define RPC_CHUNK_SIZE 60      // Bytes per chunk
#define RPC_CHUNK_MAX_COUNT 5  // Maximum chunks
#define RPC_CHUNK_BUFFER_SIZE \
  (RPC_CHUNK_SIZE * RPC_CHUNK_MAX_COUNT)  // 300 bytes

// Set backlight/contrast intensity
typedef struct {
  uint8_t nonce;
  uint8_t intensity;  // 0-255: 0 = off, 255 = max
} __PACKED SetBacklightIntensityRequest;

// Set animation index and composite modes
typedef struct {
  uint8_t nonce;
  GPUAnimationIndex animation_index;
  // Composite modes only applicable to GPU_ANIMATION_FRAMEBUFFER
  PreCompositeMode pre_composite_mode;
  PostCompositeMode post_composite_mode;
} __PACKED SetAnimationIndexRequest;

// Set animation-specific data
typedef struct {
  union {
    uint32_t word[2];
    GPUTestPatternIndex test_pattern;
    uint8_t port_power_usage[8];  // For VU meters
    GOLDensityLevel gol_density;  // For Game of Life (LOW/MEDIUM/HIGH)
  } data;
} __PACKED SetAnimationDataRequest;

// Draw text to framebuffer
typedef struct {
  int8_t x;                // X coordinate (can be negative for off-screen)
  int8_t y;                // Y coordinate (can be negative for off-screen)
  GPUTextRotation rotation;  // Text rotation (0°/90°/180°/270°) - controls glyph and flow
  uint8_t font_size;       // 0: SMALL, 1: LARGE (GPUFontSizeIndex)
  uint8_t length;          // Text length (max 12)
  uint8_t text[12];        // ASCII text
  BitBLTMode bitblt_mode;
  PreCompositeMode pre_composite_mode;
  PostCompositeMode post_composite_mode;
} __PACKED DrawTextRequest;

// Prepare for framebuffer upload (multi-chunk)
typedef struct {
  uint8_t compressed;   // 0: uncompressed, 1: RLE compressed
  uint8_t chunk_count;  // Total chunks (1 to RPC_CHUNK_MAX_COUNT)
  int8_t x;             // X coordinate (can be negative for off-screen)
  int8_t y;             // Y coordinate (can be negative for off-screen)
  uint8_t width;        // 0 to PANEL_WIDTH
  uint8_t height;       // 0 to PANEL_HEIGHT
  BitBLTMode bitblt_mode;
  PreCompositeMode pre_composite_mode;
  PostCompositeMode post_composite_mode;
} __PACKED PrepareFramebufferRequest;

// Upload framebuffer chunk data
typedef struct {
  uint8_t chunk_index;                 // 0 to RPC_CHUNK_MAX_COUNT-1
  uint8_t chunk_data[RPC_CHUNK_SIZE];  // Payload data
} __PACKED SetFramebufferRequest;

// Set display power status
typedef struct {
  uint8_t on;  // 0: off, 1: on
} __PACKED SetDisplayStatusRequest;

#endif  // !GPU_DATA_TYPES_H_
