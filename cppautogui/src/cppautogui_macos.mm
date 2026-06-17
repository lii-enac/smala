#include "cppautogui.hpp"

#include <ApplicationServices/ApplicationServices.h>
#include <CoreFoundation/CoreFoundation.h>
#include <ImageIO/ImageIO.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;

namespace cppautogui {

bool screenshot_window_or_region_bmp(const fs::path& output,
                                     const std::string& app_window_name,
                                     Rect fallback_region,
                                     int scale,
                                     std::string* error);

namespace {

constexpr int kEscapeKeyCode = 53;

struct BmpImage {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> rgba;
};

struct WindowInfo {
  Rect bounds;
  CGWindowID id = kCGNullWindowID;
};

std::string cf_string_to_std(CFStringRef value)
{
  if (value == nullptr) {
    return {};
  }
  char small[512];
  if (CFStringGetCString(value, small, sizeof(small), kCFStringEncodingUTF8)) {
    return small;
  }
  const CFIndex len = CFStringGetLength(value);
  const CFIndex max_size = CFStringGetMaximumSizeForEncoding(len, kCFStringEncodingUTF8) + 1;
  std::vector<char> buffer(static_cast<size_t>(max_size));
  if (CFStringGetCString(value, buffer.data(), max_size, kCFStringEncodingUTF8)) {
    return buffer.data();
  }
  return {};
}

std::string path_to_utf8(const fs::path& path)
{
  return path.lexically_normal().string();
}

void set_error(std::string* error, const std::string& message)
{
  if (error != nullptr) {
    *error = message;
  }
}

bool create_parent_dir(const fs::path& path, std::string* error = nullptr)
{
  const fs::path parent = path.parent_path();
  if (parent.empty()) {
    return true;
  }
  std::error_code ec;
  fs::create_directories(parent, ec);
  if (ec) {
    set_error(error, "cannot create directory " + parent.string() + ": " + ec.message());
    return false;
  }
  return true;
}

uint16_t read_u16(const std::vector<uint8_t>& bytes, size_t offset)
{
  return static_cast<uint16_t>(bytes[offset] | (bytes[offset + 1] << 8));
}

uint32_t read_u32(const std::vector<uint8_t>& bytes, size_t offset)
{
  return static_cast<uint32_t>(bytes[offset] | (bytes[offset + 1] << 8) | (bytes[offset + 2] << 16) |
                               (bytes[offset + 3] << 24));
}

int32_t read_i32(const std::vector<uint8_t>& bytes, size_t offset)
{
  return static_cast<int32_t>(read_u32(bytes, offset));
}

void write_u16(std::ostream& out, uint16_t value)
{
  out.put(static_cast<char>(value & 0xff));
  out.put(static_cast<char>((value >> 8) & 0xff));
}

void write_u32(std::ostream& out, uint32_t value)
{
  out.put(static_cast<char>(value & 0xff));
  out.put(static_cast<char>((value >> 8) & 0xff));
  out.put(static_cast<char>((value >> 16) & 0xff));
  out.put(static_cast<char>((value >> 24) & 0xff));
}

void write_i32(std::ostream& out, int32_t value)
{
  write_u32(out, static_cast<uint32_t>(value));
}

std::optional<BmpImage> load_imageio_bitmap(const fs::path& path, std::string* error)
{
  const std::string input_string = path_to_utf8(path);
  CFURLRef url = CFURLCreateFromFileSystemRepresentation(
      kCFAllocatorDefault, reinterpret_cast<const UInt8*>(input_string.c_str()), input_string.size(), false);
  if (url == nullptr) {
    set_error(error, "cannot create file URL for " + path.string());
    return std::nullopt;
  }

  CGImageSourceRef source = CGImageSourceCreateWithURL(url, nullptr);
  CFRelease(url);
  if (source == nullptr) {
    set_error(error, "ImageIO cannot open " + path.string());
    return std::nullopt;
  }

  CGImageRef cg_image = CGImageSourceCreateImageAtIndex(source, 0, nullptr);
  CFRelease(source);
  if (cg_image == nullptr) {
    set_error(error, "ImageIO cannot decode " + path.string());
    return std::nullopt;
  }

  const size_t width = CGImageGetWidth(cg_image);
  const size_t height = CGImageGetHeight(cg_image);
  if (width == 0 || height == 0 || width > static_cast<size_t>(std::numeric_limits<int>::max()) ||
      height > static_cast<size_t>(std::numeric_limits<int>::max())) {
    CGImageRelease(cg_image);
    set_error(error, "invalid image size: " + path.string());
    return std::nullopt;
  }

  BmpImage image;
  image.width = static_cast<int>(width);
  image.height = static_cast<int>(height);
  image.rgba.resize(width * height * 4);

  CGColorSpaceRef color_space = CGColorSpaceCreateDeviceRGB();
  CGContextRef context =
      CGBitmapContextCreate(image.rgba.data(), width, height, 8, width * 4, color_space,
                            kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
  if (color_space != nullptr) {
    CGColorSpaceRelease(color_space);
  }
  if (context == nullptr) {
    CGImageRelease(cg_image);
    set_error(error, "cannot create bitmap context for " + path.string());
    return std::nullopt;
  }

  CGContextDrawImage(context, CGRectMake(0.0, 0.0, static_cast<double>(width), static_cast<double>(height)),
                     cg_image);
  CGContextRelease(context);
  CGImageRelease(cg_image);
  return image;
}

std::optional<BmpImage> load_bmp(const fs::path& path, std::string* error)
{
  std::string imageio_error;
  if (std::optional<BmpImage> image = load_imageio_bitmap(path, &imageio_error)) {
    return image;
  }

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    set_error(error, "cannot open " + path.string());
    return std::nullopt;
  }
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (bytes.size() < 54 || bytes[0] != 'B' || bytes[1] != 'M') {
    set_error(error, "not a BMP file: " + path.string());
    return std::nullopt;
  }

  const uint32_t pixel_offset = read_u32(bytes, 10);
  const uint32_t dib_size = read_u32(bytes, 14);
  if (dib_size < 40 || bytes.size() < 14 + dib_size) {
    set_error(error, "unsupported BMP header: " + path.string());
    return std::nullopt;
  }

  const int32_t raw_width = read_i32(bytes, 18);
  const int32_t raw_height = read_i32(bytes, 22);
  const uint16_t planes = read_u16(bytes, 26);
  const uint16_t bits_per_pixel = read_u16(bytes, 28);
  const uint32_t compression = read_u32(bytes, 30);
  const bool supported_compression = compression == 0 || (compression == 3 && bits_per_pixel == 32);
  if (planes != 1 || !supported_compression || (bits_per_pixel != 24 && bits_per_pixel != 32) || raw_width <= 0 ||
      raw_height == 0) {
    set_error(error, "unsupported BMP format: " + path.string() + " (" + imageio_error + ")");
    return std::nullopt;
  }

  const int width = raw_width;
  const int height = raw_height < 0 ? -raw_height : raw_height;
  const bool top_down = raw_height < 0;
  const size_t bytes_per_pixel = bits_per_pixel / 8;
  const size_t row_stride = ((static_cast<size_t>(width) * bits_per_pixel + 31) / 32) * 4;
  const size_t required = static_cast<size_t>(pixel_offset) + row_stride * static_cast<size_t>(height);
  if (bytes.size() < required) {
    set_error(error, "truncated BMP data: " + path.string());
    return std::nullopt;
  }

  BmpImage image;
  image.width = width;
  image.height = height;
  image.rgba.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 4);

  for (int y = 0; y < height; ++y) {
    const int source_y = top_down ? y : (height - 1 - y);
    const size_t source = static_cast<size_t>(pixel_offset) + static_cast<size_t>(source_y) * row_stride;
    for (int x = 0; x < width; ++x) {
      const size_t src = source + static_cast<size_t>(x) * bytes_per_pixel;
      const size_t dst = (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 4;
      image.rgba[dst + 0] = bytes[src + 2];
      image.rgba[dst + 1] = bytes[src + 1];
      image.rgba[dst + 2] = bytes[src + 0];
      image.rgba[dst + 3] = bits_per_pixel == 32 ? bytes[src + 3] : 255;
    }
  }
  return image;
}

bool save_bmp(const fs::path& path, const BmpImage& image, std::string* error)
{
  if (image.width <= 0 || image.height <= 0 ||
      image.rgba.size() != static_cast<size_t>(image.width) * static_cast<size_t>(image.height) * 4) {
    set_error(error, "invalid image for " + path.string());
    return false;
  }
  if (!create_parent_dir(path, error)) {
    return false;
  }

  std::ofstream out(path, std::ios::binary);
  if (!out) {
    set_error(error, "cannot write " + path.string());
    return false;
  }

  const uint32_t row_stride = static_cast<uint32_t>(image.width) * 4;
  const uint32_t pixel_size = row_stride * static_cast<uint32_t>(image.height);
  const uint32_t file_size = 54 + pixel_size;

  out.put('B');
  out.put('M');
  write_u32(out, file_size);
  write_u16(out, 0);
  write_u16(out, 0);
  write_u32(out, 54);

  write_u32(out, 40);
  write_i32(out, image.width);
  write_i32(out, image.height);
  write_u16(out, 1);
  write_u16(out, 32);
  write_u32(out, 0);
  write_u32(out, pixel_size);
  write_i32(out, 3780);
  write_i32(out, 3780);
  write_u32(out, 0);
  write_u32(out, 0);

  for (int y = image.height - 1; y >= 0; --y) {
    for (int x = 0; x < image.width; ++x) {
      const size_t src = (static_cast<size_t>(y) * static_cast<size_t>(image.width) + static_cast<size_t>(x)) * 4;
      out.put(static_cast<char>(image.rgba[src + 2]));
      out.put(static_cast<char>(image.rgba[src + 1]));
      out.put(static_cast<char>(image.rgba[src + 0]));
      out.put(static_cast<char>(image.rgba[src + 3]));
    }
  }
  return static_cast<bool>(out);
}

CGMouseButton to_cg_button(MouseButton button)
{
  switch (button) {
    case MouseButton::Right:
      return kCGMouseButtonRight;
    case MouseButton::Middle:
      return kCGMouseButtonCenter;
    case MouseButton::Left:
    default:
      return kCGMouseButtonLeft;
  }
}

CGEventType mouse_down_type(MouseButton button)
{
  switch (button) {
    case MouseButton::Right:
      return kCGEventRightMouseDown;
    case MouseButton::Middle:
      return kCGEventOtherMouseDown;
    case MouseButton::Left:
    default:
      return kCGEventLeftMouseDown;
  }
}

CGEventType mouse_up_type(MouseButton button)
{
  switch (button) {
    case MouseButton::Right:
      return kCGEventRightMouseUp;
    case MouseButton::Middle:
      return kCGEventOtherMouseUp;
    case MouseButton::Left:
    default:
      return kCGEventLeftMouseUp;
  }
}

CGEventType mouse_drag_type(MouseButton button)
{
  switch (button) {
    case MouseButton::Right:
      return kCGEventRightMouseDragged;
    case MouseButton::Middle:
      return kCGEventOtherMouseDragged;
    case MouseButton::Left:
    default:
      return kCGEventLeftMouseDragged;
  }
}

void post_mouse(CGEventType type, Point point, MouseButton button)
{
  CGEventRef event = CGEventCreateMouseEvent(nullptr, type, CGPointMake(point.x, point.y), to_cg_button(button));
  if (event == nullptr) {
    return;
  }
  CGEventPost(kCGHIDEventTap, event);
  CFRelease(event);
}

double ease_out_quad(double value)
{
  return 1.0 - (1.0 - value) * (1.0 - value);
}

void interpolate(Point target, MouseButton button, double duration_seconds, bool dragged)
{
  const Point start = mouse_position();
  if (duration_seconds <= 0.0) {
    post_mouse(dragged ? mouse_drag_type(button) : kCGEventMouseMoved, target, button);
    return;
  }

  const int steps = std::max(1, static_cast<int>(std::ceil(duration_seconds * 60.0)));
  const auto step_delay = std::chrono::duration<double>(duration_seconds / steps);
  for (int i = 1; i <= steps; ++i) {
    const double t = ease_out_quad(static_cast<double>(i) / static_cast<double>(steps));
    const Point p{start.x + (target.x - start.x) * t, start.y + (target.y - start.y) * t};
    post_mouse(dragged ? mouse_drag_type(button) : kCGEventMouseMoved, p, button);
    std::this_thread::sleep_for(step_delay);
  }
}

MouseButton event_button(CGEventType type, CGEventRef event)
{
  if (type == kCGEventRightMouseDown || type == kCGEventRightMouseUp || type == kCGEventRightMouseDragged) {
    return MouseButton::Right;
  }
  if (type == kCGEventOtherMouseDown || type == kCGEventOtherMouseUp || type == kCGEventOtherMouseDragged) {
    const int64_t button = CGEventGetIntegerValueField(event, kCGMouseEventButtonNumber);
    if (button == kCGMouseButtonRight) {
      return MouseButton::Right;
    }
    return MouseButton::Middle;
  }
  return MouseButton::Left;
}

std::string key_name(CGEventRef event)
{
  const auto keycode = static_cast<int>(CGEventGetIntegerValueField(event, kCGKeyboardEventKeycode));
  if (keycode == kEscapeKeyCode) {
    return "Key.esc";
  }

  UniChar chars[8] = {};
  UniCharCount length = 0;
  CGEventKeyboardGetUnicodeString(event, 8, &length, chars);
  if (length > 0 && chars[0] >= 32 && chars[0] < 127) {
    return std::string(1, static_cast<char>(chars[0]));
  }

  return "KeyCode." + std::to_string(keycode);
}

bool launch_make_test(const std::string& app_name)
{
  const std::string target = app_name + "_test";
  const pid_t pid = fork();
  if (pid < 0) {
    return false;
  }
  if (pid == 0) {
    chdir("..");
    const int dev_null = open("/dev/null", O_WRONLY);
    if (dev_null >= 0) {
      dup2(dev_null, STDOUT_FILENO);
      dup2(dev_null, STDERR_FILENO);
      close(dev_null);
    }
    execlp("make", "make", "-j", target.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  return true;
}

CGWindowImageOption window_image_options(int scale)
{
  if (scale > 1) {
    return static_cast<CGWindowImageOption>(kCGWindowImageBoundsIgnoreFraming | kCGWindowImageBestResolution);
  }
  return static_cast<CGWindowImageOption>(kCGWindowImageBoundsIgnoreFraming | kCGWindowImageNominalResolution);
}

CGImageRef create_window_screenshot_image(CGWindowID window_id, int scale)
{
  using WindowImageFn = CGImageRef (*)(CGRect, CGWindowListOption, CGWindowID, CGWindowImageOption);
  auto* raw_symbol = dlsym(RTLD_DEFAULT, "CGWindowListCreateImage");
  if (raw_symbol == nullptr || window_id == kCGNullWindowID) {
    return nullptr;
  }

  auto* window_image = reinterpret_cast<WindowImageFn>(raw_symbol);
  return window_image(CGRectNull, kCGWindowListOptionIncludingWindow, window_id, window_image_options(scale));
}

CGImageRef create_screenshot_image(CGRect capture_rect, int scale)
{
  using WindowImageFn = CGImageRef (*)(CGRect, CGWindowListOption, CGWindowID, CGWindowImageOption);
  using DisplayImageFn = CGImageRef (*)(CGDirectDisplayID, CGRect);
  auto* raw_symbol = dlsym(RTLD_DEFAULT, "CGWindowListCreateImage");
  if (raw_symbol != nullptr) {
    auto* window_image = reinterpret_cast<WindowImageFn>(raw_symbol);
    if (CGImageRef image =
            window_image(capture_rect, kCGWindowListOptionOnScreenOnly, kCGNullWindowID, window_image_options(scale))) {
      return image;
    }
  }

  auto* raw_display_symbol = dlsym(RTLD_DEFAULT, "CGDisplayCreateImageForRect");
  if (raw_display_symbol == nullptr) {
    return nullptr;
  }
  auto* display_image = reinterpret_cast<DisplayImageFn>(raw_display_symbol);

  uint32_t display_count = 0;
  if (CGGetActiveDisplayList(0, nullptr, &display_count) != kCGErrorSuccess || display_count == 0) {
    return nullptr;
  }
  std::vector<CGDirectDisplayID> displays(display_count);
  if (CGGetActiveDisplayList(display_count, displays.data(), &display_count) != kCGErrorSuccess) {
    return nullptr;
  }

  const CGPoint center =
      CGPointMake(CGRectGetMidX(capture_rect), CGRectGetMidY(capture_rect));
  for (CGDirectDisplayID display : displays) {
    const CGRect bounds = CGDisplayBounds(display);
    if (!CGRectContainsPoint(bounds, center)) {
      continue;
    }
    const double scale_x = bounds.size.width == 0.0 ? 1.0 : CGDisplayPixelsWide(display) / bounds.size.width;
    const double scale_y = bounds.size.height == 0.0 ? 1.0 : CGDisplayPixelsHigh(display) / bounds.size.height;
    const CGRect local_rect =
        CGRectMake((capture_rect.origin.x - bounds.origin.x) * (scale > 1 ? scale_x : 1.0),
                   (capture_rect.origin.y - bounds.origin.y) * (scale > 1 ? scale_y : 1.0),
                   capture_rect.size.width * (scale > 1 ? scale_x : 1.0),
                   capture_rect.size.height * (scale > 1 ? scale_y : 1.0));
    return display_image(display, local_rect);
  }

  return display_image(CGMainDisplayID(), capture_rect);
}

void remove_matching_bmps(const fs::path& dir, const std::string& prefix)
{
  std::error_code ec;
  if (!fs::exists(dir, ec)) {
    return;
  }
  for (const auto& entry : fs::directory_iterator(dir, ec)) {
    if (ec) {
      return;
    }
    if (!entry.is_regular_file()) {
      continue;
    }
    const fs::path path = entry.path();
    const std::string name = path.filename().string();
    if (path.extension() == ".bmp" && (prefix.empty() || name.rfind(prefix, 0) == 0)) {
      fs::remove(path, ec);
    }
  }
}

std::optional<WindowInfo> find_window_info_by_name(const std::string& app_name)
{
  CFArrayRef windows = CGWindowListCopyWindowInfo(kCGWindowListOptionOnScreenOnly, kCGNullWindowID);
  if (windows == nullptr) {
    return std::nullopt;
  }

  std::optional<WindowInfo> result;
  const CFIndex count = CFArrayGetCount(windows);
  for (CFIndex i = 0; i < count; ++i) {
    auto* dict = static_cast<CFDictionaryRef>(CFArrayGetValueAtIndex(windows, i));
    auto* owner = static_cast<CFStringRef>(CFDictionaryGetValue(dict, kCGWindowOwnerName));
    if (cf_string_to_std(owner) != app_name) {
      continue;
    }

    int layer = 0;
    if (auto* layer_number = static_cast<CFNumberRef>(CFDictionaryGetValue(dict, kCGWindowLayer))) {
      CFNumberGetValue(layer_number, kCFNumberIntType, &layer);
    }
    if (layer != 0) {
      continue;
    }

    auto* bounds = static_cast<CFDictionaryRef>(CFDictionaryGetValue(dict, kCGWindowBounds));
    CGRect rect = CGRectNull;
    if (bounds == nullptr || !CGRectMakeWithDictionaryRepresentation(bounds, &rect) || rect.size.width <= 0 ||
        rect.size.height <= 0) {
      continue;
    }

    uint32_t window_number = 0;
    if (auto* window_number_ref = static_cast<CFNumberRef>(CFDictionaryGetValue(dict, kCGWindowNumber))) {
      CFNumberGetValue(window_number_ref, kCFNumberSInt32Type, &window_number);
    }
    if (window_number == 0) {
      continue;
    }

    result = WindowInfo{Rect{rect.origin.x, rect.origin.y, rect.size.width, rect.size.height}, window_number};
    break;
  }

  CFRelease(windows);
  return result;
}

std::string retina_suffix(bool retina)
{
  return retina ? "_Retina" : "";
}

fs::path screenshot_result_path(const fs::path& dir, const std::string& app_name, int index, bool retina)
{
  return dir / (app_name + "_" + std::to_string(index) + retina_suffix(retina) + ".bmp");
}

fs::path screenshot_ref_path(const fs::path& dir, const std::string& app_name, int index, bool retina)
{
  return dir / (app_name + "_" + std::to_string(index) + retina_suffix(retina) + "_Ref.bmp");
}

fs::path screenshot_diff_path(const fs::path& dir, const std::string& app_name, int index, bool retina)
{
  return dir / (app_name + "_" + std::to_string(index) + retina_suffix(retina) + "_DIFF.bmp");
}

bool compare_screenshots(const ReplayOptions& options, int max_screenshot)
{
  bool result = true;
  const fs::path diff_dir = options.result_dir / "diffs";
  for (int i = 1; i <= max_screenshot; ++i) {
    const fs::path ref = screenshot_ref_path(options.ref_dir, options.app_name, i, options.retina);
    const fs::path got = screenshot_result_path(options.result_dir, options.app_name, i, options.retina);
    const fs::path diff = screenshot_diff_path(diff_dir, options.app_name, i, options.retina);
    const ImageDiff image_diff = compare_bmp(ref, got, diff, options.diff_threshold_percent);
    if (!image_diff.error.empty()) {
      std::cerr << "\033[0;31m [KO] " << image_diff.error << "\033[0m\n";
      result = false;
      continue;
    }
    std::cout << "diff_ratio: " << image_diff.ratio_percent << "\n";
    if (image_diff.ok) {
      std::cout << "\033[0;32m [OK] - " << ref << " - " << got << "\033[0m\n";
    } else {
      std::cout << "\033[0;31m WARNING --- images are DIFFERENT - those differences have been saved in: "
                << diff << "\033[0m\n";
      result = false;
    }
  }
  return result;
}

struct RecorderState {
  RecordOptions options;
  std::ofstream log;
  int screenshot_counter = 0;
  bool ctrl_alt_down = false;
  CFMachPortRef event_tap = nullptr;
  CFRunLoopRef run_loop = nullptr;
};

void record_screenshot(RecorderState& state)
{
  ++state.screenshot_counter;
  const std::string window_name = state.options.app_name + "_app";
  const std::optional<WindowInfo> window = find_window_info_by_name(window_name);
  if (!window) {
    std::cerr << "\tWARNING - '" << state.options.app_name << "' window NOT FOUND\n";
    if (state.run_loop != nullptr) {
      CFRunLoopStop(state.run_loop);
    }
    return;
  }

  const Point pos = mouse_position();
  const fs::path screenshot_name =
      state.options.save_dir /
      (state.options.app_name + "_" + std::to_string(state.screenshot_counter) + retina_suffix(state.options.retina) +
       "_Ref.bmp");
  std::string error;
  const int scale = state.options.retina ? 2 : 1;
  if (!screenshot_window_or_region_bmp(screenshot_name, window_name, window->bounds, scale, &error)) {
    std::cerr << "cannot save screenshot: " << error << "\n";
  }
  if (state.options.debug) {
    std::cout << screenshot_name << " " << window->bounds.x << " " << window->bounds.y << " "
              << window->bounds.width << " " << window->bounds.height << " " << pos.x << " " << pos.y
              << "\n";
  }
  state.log << "screenshot " << state.screenshot_counter << " " << window->bounds.x << " " << window->bounds.y << " "
            << window->bounds.width << " " << window->bounds.height << " " << static_cast<int>(pos.x) << " "
            << static_cast<int>(pos.y) << "\n";
  state.log.flush();
}

CGEventRef record_callback(CGEventTapProxy, CGEventType type, CGEventRef event, void* user_info)
{
  auto* state = static_cast<RecorderState*>(user_info);
  if (state == nullptr) {
    return event;
  }

  if (type == kCGEventTapDisabledByTimeout || type == kCGEventTapDisabledByUserInput) {
    if (state->event_tap != nullptr) {
      CGEventTapEnable(state->event_tap, true);
    }
    return event;
  }

  if (type == kCGEventFlagsChanged) {
    const CGEventFlags flags = CGEventGetFlags(event);
    const bool ctrl_alt = (flags & kCGEventFlagMaskControl) != 0 && (flags & kCGEventFlagMaskAlternate) != 0;
    if (ctrl_alt && !state->ctrl_alt_down) {
      record_screenshot(*state);
    }
    state->ctrl_alt_down = ctrl_alt;
    return event;
  }

  if (type == kCGEventKeyDown) {
    const std::string name = key_name(event);
    state->log << "k " << name << "\n";
    state->log.flush();
    if (state->options.debug) {
      std::cout << "k " << name << "\n";
    }
    if (name == "Key.esc") {
      state->log << "quit x x\n";
      state->log.flush();
      if (state->run_loop != nullptr) {
        CFRunLoopStop(state->run_loop);
      }
    }
    return event;
  }

  if (type == kCGEventMouseMoved || type == kCGEventLeftMouseDragged || type == kCGEventRightMouseDragged ||
      type == kCGEventOtherMouseDragged) {
    const CGPoint p = CGEventGetLocation(event);
    state->log << "m " << static_cast<int>(p.x) << " " << static_cast<int>(p.y) << "\n";
    if (state->options.debug) {
      std::cout << "m " << static_cast<int>(p.x) << " " << static_cast<int>(p.y) << "\n";
    }
    return event;
  }

  if (type == kCGEventLeftMouseDown || type == kCGEventRightMouseDown || type == kCGEventOtherMouseDown ||
      type == kCGEventLeftMouseUp || type == kCGEventRightMouseUp || type == kCGEventOtherMouseUp) {
    const CGPoint p = CGEventGetLocation(event);
    const bool pressed =
        type == kCGEventLeftMouseDown || type == kCGEventRightMouseDown || type == kCGEventOtherMouseDown;
    state->log << (pressed ? "p " : "r ") << static_cast<int>(p.x) << " " << static_cast<int>(p.y) << " "
               << button_to_log(event_button(type, event)) << "\n";
    if (state->options.debug) {
      std::cout << (pressed ? "p " : "r ") << static_cast<int>(p.x) << " " << static_cast<int>(p.y) << "\n";
    }
    return event;
  }

  return event;
}

std::vector<std::string> list_test_names(const fs::path& data_dir)
{
  std::set<std::string> names;
  std::error_code ec;
  if (!fs::exists(data_dir, ec)) {
    return {};
  }
  for (const auto& entry : fs::directory_iterator(data_dir, ec)) {
    if (ec || !entry.is_regular_file()) {
      continue;
    }
    const std::string filename = entry.path().filename().string();
    const size_t marker = filename.find("_data");
    if (marker != std::string::npos) {
      names.insert(filename.substr(0, marker));
    }
  }
  return {names.begin(), names.end()};
}

} // namespace

std::string button_to_log(MouseButton button)
{
  switch (button) {
    case MouseButton::Right:
      return "Button.right";
    case MouseButton::Middle:
      return "Button.middle";
    case MouseButton::Left:
    default:
      return "Button.left";
  }
}

std::optional<MouseButton> parse_button(const std::string& text)
{
  if (text == "Button.left" || text == "left") {
    return MouseButton::Left;
  }
  if (text == "Button.right" || text == "right") {
    return MouseButton::Right;
  }
  if (text == "Button.middle" || text == "middle" || text == "Button.center") {
    return MouseButton::Middle;
  }
  return std::nullopt;
}

std::optional<Rect> find_window_geometry_by_name(const std::string& app_name)
{
  const std::optional<WindowInfo> info = find_window_info_by_name(app_name);
  if (!info) {
    return std::nullopt;
  }
  return info->bounds;
}

Point mouse_position()
{
  CGEventRef event = CGEventCreate(nullptr);
  if (event == nullptr) {
    return {};
  }
  const CGPoint p = CGEventGetLocation(event);
  CFRelease(event);
  return Point{p.x, p.y};
}

void move_to(Point point, double duration_seconds)
{
  interpolate(point, MouseButton::Left, duration_seconds, false);
}

void drag_to(Point point, MouseButton button, double duration_seconds)
{
  interpolate(point, button, duration_seconds, true);
}

void mouse_down(Point point, MouseButton button)
{
  post_mouse(mouse_down_type(button), point, button);
}

void mouse_up(Point point, MouseButton button)
{
  post_mouse(mouse_up_type(button), point, button);
}

bool save_cgimage_as_bmp(const fs::path& output, CGImageRef image, std::string* error)
{
  if (!create_parent_dir(output, error)) {
    return false;
  }

  if (image == nullptr) {
    set_error(error, "screen capture failed");
    return false;
  }

  const std::string output_string = path_to_utf8(output);
  CFURLRef url = CFURLCreateFromFileSystemRepresentation(
      kCFAllocatorDefault, reinterpret_cast<const UInt8*>(output_string.c_str()), output_string.size(), false);
  if (url == nullptr) {
    set_error(error, "cannot create file URL for " + output.string());
    return false;
  }

  CGImageDestinationRef destination =
      CGImageDestinationCreateWithURL(url, CFSTR("com.microsoft.bmp"), 1, nullptr);
  if (destination == nullptr) {
    CFRelease(url);
    set_error(error, "cannot create BMP destination for " + output.string());
    return false;
  }

  CGImageDestinationAddImage(destination, image, nullptr);
  const bool ok = CGImageDestinationFinalize(destination);
  CFRelease(destination);
  CFRelease(url);

  if (!ok) {
    set_error(error, "cannot write BMP " + output.string());
    return false;
  }
  return true;
}

bool screenshot_bmp(const fs::path& output, Rect region, int scale, std::string* error)
{
  const CGRect capture_rect = CGRectMake(region.x, region.y, region.width, region.height);
  CGImageRef image = create_screenshot_image(capture_rect, scale);
  const bool ok = save_cgimage_as_bmp(output, image, error);
  if (image != nullptr) {
    CGImageRelease(image);
  }
  return ok;
}

bool screenshot_window_or_region_bmp(const fs::path& output,
                                     const std::string& app_window_name,
                                     Rect fallback_region,
                                     int scale,
                                     std::string* error)
{
  if (const std::optional<WindowInfo> window = find_window_info_by_name(app_window_name)) {
    CGImageRef image = create_window_screenshot_image(window->id, scale);
    if (image != nullptr) {
      const bool ok = save_cgimage_as_bmp(output, image, error);
      CGImageRelease(image);
      return ok;
    }
  }
  return screenshot_bmp(output, fallback_region, scale, error);
}

ImageDiff compare_bmp(const fs::path& reference,
                      const fs::path& result,
                      const fs::path& diff_output,
                      double threshold_percent)
{
  std::string error;
  const std::optional<BmpImage> ref = load_bmp(reference, &error);
  if (!ref) {
    return ImageDiff{false, 100.0, error};
  }
  const std::optional<BmpImage> got = load_bmp(result, &error);
  if (!got) {
    return ImageDiff{false, 100.0, error};
  }
  if (ref->width != got->width || ref->height != got->height) {
    std::ostringstream message;
    message << "image sizes differ: " << reference << " is " << ref->width << "x" << ref->height << ", "
            << result << " is " << got->width << "x" << got->height;
    return ImageDiff{false, 100.0, message.str()};
  }

  BmpImage diff;
  diff.width = ref->width;
  diff.height = ref->height;
  diff.rgba.resize(ref->rgba.size());

  double total = 0.0;
  const size_t pixels = static_cast<size_t>(ref->width) * static_cast<size_t>(ref->height);
  for (size_t i = 0; i < pixels; ++i) {
    const size_t offset = i * 4;
    for (int c = 0; c < 3; ++c) {
      const auto delta = static_cast<uint8_t>(
          std::abs(static_cast<int>(ref->rgba[offset + c]) - static_cast<int>(got->rgba[offset + c])));
      diff.rgba[offset + c] = delta;
      total += delta;
    }
    diff.rgba[offset + 3] = 255;
  }

  const double ratio = pixels == 0 ? 100.0 : (total / (static_cast<double>(pixels) * 3.0 * 255.0) * 100.0);
  const bool ok = ratio <= threshold_percent;
  if (!ok) {
    std::string save_error;
    save_bmp(diff_output, diff, &save_error);
  }
  return ImageDiff{ok, ratio, {}};
}

int run_record(const RecordOptions& options)
{
  std::error_code ec;
  fs::create_directories(options.save_dir, ec);
  if (ec) {
    std::cerr << "cannot create " << options.save_dir << ": " << ec.message() << "\n";
    return 1;
  }

  const fs::path datafile = options.save_dir / (options.app_name + "_data.txt");
  RecorderState state;
  state.options = options;
  state.log.open(datafile, std::ios::trunc);
  if (!state.log) {
    std::cerr << "cannot write " << datafile << "\n";
    return 1;
  }

  std::cout << "------------ RECORD -------------\n";
  std::cout << "test name:\t " << options.app_name << "\n";
  std::cout << "Data file:\t " << datafile << "\n";
  std::cout << "debug_mode:\t " << options.debug << "\n";
  std::cout << "Retina display:\t " << options.retina << "\n\n";
  std::cout << "WARNING:\n";
  std::cout << "all files will be saved in: " << options.save_dir << "\n";
  std::cout << "copy them to datas/ and references/ when they become the baseline.\n\n";

  if (options.launch_test) {
    std::cout << "launching test: " << options.app_name << "_test\n";
    if (!launch_make_test(options.app_name)) {
      std::cerr << "cannot launch make target\n";
    }
  }

  std::cout << "\nPress CTRL+ALT to make screenshot\n";
  std::cout << "Press ESC to Quit\n";
  std::cout << "---------------------------------\n\n";

  const CGEventMask mask = CGEventMaskBit(kCGEventFlagsChanged) | CGEventMaskBit(kCGEventKeyDown) |
                           CGEventMaskBit(kCGEventMouseMoved) | CGEventMaskBit(kCGEventLeftMouseDragged) |
                           CGEventMaskBit(kCGEventRightMouseDragged) | CGEventMaskBit(kCGEventOtherMouseDragged) |
                           CGEventMaskBit(kCGEventLeftMouseDown) | CGEventMaskBit(kCGEventRightMouseDown) |
                           CGEventMaskBit(kCGEventOtherMouseDown) | CGEventMaskBit(kCGEventLeftMouseUp) |
                           CGEventMaskBit(kCGEventRightMouseUp) | CGEventMaskBit(kCGEventOtherMouseUp);

  state.event_tap = CGEventTapCreate(kCGSessionEventTap, kCGHeadInsertEventTap, kCGEventTapOptionListenOnly, mask,
                                     record_callback, &state);
  if (state.event_tap == nullptr) {
    std::cerr << "cannot create macOS event tap. Check Accessibility and Input Monitoring permissions for Terminal.\n";
    return 1;
  }

  CFRunLoopSourceRef source = CFMachPortCreateRunLoopSource(kCFAllocatorDefault, state.event_tap, 0);
  if (source == nullptr) {
    CFRelease(state.event_tap);
    std::cerr << "cannot create run loop source\n";
    return 1;
  }

  state.run_loop = CFRunLoopGetCurrent();
  CFRunLoopAddSource(state.run_loop, source, kCFRunLoopCommonModes);
  CGEventTapEnable(state.event_tap, true);
  CFRunLoopRun();
  CFRunLoopRemoveSource(state.run_loop, source, kCFRunLoopCommonModes);
  CFRelease(source);
  CFRelease(state.event_tap);
  return 0;
}

int run_replay(const ReplayOptions& input_options)
{
  ReplayOptions options = input_options;
  if (options.test_mode) {
    options.data_dir = "new_records";
    options.ref_dir = "new_records";
  }

  const fs::path nointer_file = options.data_dir / (options.app_name + "_data_nointer.txt");
  fs::path datafile = options.data_dir / (options.app_name + "_data.txt");
  bool interpolate = !options.no_interpolation;
  if (fs::exists(nointer_file)) {
    datafile = nointer_file;
    interpolate = false;
  }

  std::cout << "\n------------ REPLAY " << options.app_name << " -------------\n";
  std::cout << "test name:\t\t " << options.app_name << "\n";
  std::cout << "Data file:\t\t " << datafile << "\n";
  std::cout << "interpolation:\t\t " << interpolate << "\n";
  std::cout << "Retina display:\t\t " << options.retina << "\n\n";

  if (!fs::exists(datafile)) {
    std::cerr << "missing data file: " << datafile << "\n";
    return 1;
  }

  std::error_code ec;
  fs::create_directories(options.result_dir, ec);
  ec.clear();
  fs::create_directories(options.result_dir / "diffs", ec);
  std::cout << "Erasing: " << options.app_name << " results and diffs\n";
  remove_matching_bmps(options.result_dir, options.app_name);
  remove_matching_bmps(options.result_dir / "diffs", options.app_name);

  if (options.launch_test) {
    std::cout << "launching test: " << options.app_name << "_test\n";
    launch_make_test(options.app_name);
    std::this_thread::sleep_for(std::chrono::duration<double>(options.wait_seconds));
  }
  std::cout << "---------------------------------\n\n";

  std::ifstream data(datafile);
  if (!data) {
    std::cerr << "cannot open " << datafile << "\n";
    return 1;
  }

  bool pressed = false;
  MouseButton button = MouseButton::Left;
  int max_screenshot = 0;
  std::string line;
  while (std::getline(data, line)) {
    std::istringstream iss(line);
    std::string type;
    iss >> type;
    if (type.empty()) {
      continue;
    }

    if (type == "m") {
      double x = 0.0;
      double y = 0.0;
      if (iss >> x >> y) {
        if (!interpolate) {
          if (pressed) {
            drag_to(Point{x, y}, button, 0.0);
          } else {
            move_to(Point{x, y}, 0.0);
          }
        }
      }
      continue;
    }

    if (type == "p" || type == "r") {
      double x = 0.0;
      double y = 0.0;
      std::string raw_button;
      if (!(iss >> x >> y)) {
        continue;
      }
      if (iss >> raw_button) {
        button = parse_button(raw_button).value_or(MouseButton::Left);
      } else {
        button = MouseButton::Left;
      }

      if (type == "p") {
        pressed = true;
        mouse_down(Point{x, y}, button);
        if (interpolate) {
          move_to(Point{x, y}, 1.0);
        }
      } else {
        if (interpolate) {
          drag_to(Point{x, y}, button, 1.0);
        }
        pressed = false;
        mouse_up(Point{x, y}, button);
      }
      continue;
    }

    if (type == "screenshot") {
      int index = 0;
      Rect region;
      Point mouse;
      if (!(iss >> index >> region.x >> region.y >> region.width >> region.height >> mouse.x >> mouse.y)) {
        continue;
      }
      max_screenshot = std::max(max_screenshot, index);
      if (interpolate) {
        if (pressed) {
          drag_to(mouse, button, 1.0);
        } else {
          move_to(mouse, 1.0);
        }
      }
      const fs::path output = screenshot_result_path(options.result_dir, options.app_name, index, options.retina);
      std::string error;
      if (!screenshot_window_or_region_bmp(output, options.app_name + "_app", region, options.retina ? 2 : 1,
                                           &error)) {
        std::cerr << "cannot save screenshot " << output << ": " << error << "\n";
      }
      continue;
    }

    if (type == "k") {
      std::cout << line << "\n";
      continue;
    }

    if (type == "quit") {
      return compare_screenshots(options, max_screenshot) ? 0 : 1;
    }

    // Ignore debug lines emitted by Python/PIL in old recordings.
  }

  return compare_screenshots(options, max_screenshot) ? 0 : 1;
}

int run_launch_tests(const LaunchOptions& options)
{
  const std::vector<std::string> test_names = list_test_names(options.data_dir);
  std::vector<std::string> ok;
  std::vector<std::string> ko;

  std::cout << "\n\n------------------- ALL TESTS ----------------------\n";
  std::cout << "Retina display:\t\t " << options.retina << "\n\n";
  std::cout << "List of the tests:";
  for (const std::string& name : test_names) {
    std::cout << " " << name;
  }
  std::cout << "\n";

  std::cout << "Erasing: ALL results and diffs\n";
  remove_matching_bmps("results", "");
  remove_matching_bmps("results/diffs", "");

  for (const std::string& test : test_names) {
    std::cout << "\n\nlaunching test ... " << test << "\n";
    ReplayOptions replay;
    replay.app_name = test;
    replay.retina = options.retina;
    replay.wait_seconds = options.wait_seconds;
    replay.diff_threshold_percent = options.diff_threshold_percent;
    if (run_replay(replay) == 0) {
      ok.push_back(test);
    } else {
      ko.push_back(test);
    }
  }

  std::cout << "\n\n\033[0;32m------------- ALL TESTS RESULTS ----------------------\033[0m\n";
  for (const std::string& name : ok) {
    std::cout << "\033[0;32m [OK] \033[0m - " << name << "\n";
  }
  for (const std::string& name : ko) {
    std::cout << "\033[0;31m [KO] \033[0m - " << name << "\n";
  }
  const size_t total = ok.size() + ko.size();
  std::cout << "\033[0;32m-----------------------------------------------------\033[0m\n";
  std::cout << "\033[0;32m Total Passed: " << ok.size() << "/" << total << "\033[0m\n";
  if (!ko.empty()) {
    std::cout << "\033[0;31m Total Failed: " << ko.size() << "/" << total << "\033[0m\n";
  }
  std::cout << "\n\n";
  return ko.empty() ? 0 : 1;
}

} // namespace cppautogui
