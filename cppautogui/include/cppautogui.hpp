#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace cppautogui {

struct Point {
  double x = 0.0;
  double y = 0.0;
};

struct Rect {
  double x = 0.0;
  double y = 0.0;
  double width = 0.0;
  double height = 0.0;
};

enum class MouseButton {
  Left,
  Right,
  Middle,
};

struct RecordOptions {
  std::string app_name = "default";
  bool debug = false;
  bool retina = false;
  bool launch_test = true;
  std::filesystem::path save_dir = "new_records";
};

struct ReplayOptions {
  std::string app_name = "default";
  bool test_mode = false;
  bool retina = false;
  bool no_interpolation = false;
  bool launch_test = true;
  double wait_seconds = 2.0;
  double diff_threshold_percent = 1.0;
  std::filesystem::path data_dir = "datas";
  std::filesystem::path ref_dir = "references";
  std::filesystem::path result_dir = "results";
};

struct LaunchOptions {
  bool retina = false;
  double wait_seconds = 2.0;
  double diff_threshold_percent = 1.0;
  std::filesystem::path data_dir = "datas";
};

struct ImageDiff {
  bool ok = false;
  double ratio_percent = 0.0;
  std::string error;
};

std::string button_to_log(MouseButton button);
std::optional<MouseButton> parse_button(const std::string& text);

std::optional<Rect> find_window_geometry_by_name(const std::string& app_name);
Point mouse_position();

void move_to(Point point, double duration_seconds = 0.0);
void drag_to(Point point, MouseButton button = MouseButton::Left, double duration_seconds = 0.0);
void mouse_down(Point point, MouseButton button = MouseButton::Left);
void mouse_up(Point point, MouseButton button = MouseButton::Left);

bool screenshot_bmp(const std::filesystem::path& output, Rect region, int scale, std::string* error = nullptr);
ImageDiff compare_bmp(const std::filesystem::path& reference,
                      const std::filesystem::path& result,
                      const std::filesystem::path& diff_output,
                      double threshold_percent);

int run_record(const RecordOptions& options);
int run_replay(const ReplayOptions& options);
int run_launch_tests(const LaunchOptions& options);

} // namespace cppautogui
