#include "cppautogui.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void usage()
{
  std::cout << "launch_tests_cppautogui [-r|--retina] [--wait seconds] [--ratio percent]\n";
}

bool read_value(int& index, int argc, char** argv, std::string& value)
{
  if (index + 1 >= argc) {
    return false;
  }
  value = argv[++index];
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  cppautogui::LaunchOptions options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      usage();
      return 0;
    }
    if (arg == "-r" || arg == "--retina") {
      options.retina = true;
      continue;
    }
    if (arg == "--wait") {
      std::string value;
      if (!read_value(i, argc, argv, value)) {
        usage();
        return 2;
      }
      options.wait_seconds = std::atof(value.c_str());
      continue;
    }
    if (arg == "--ratio") {
      std::string value;
      if (!read_value(i, argc, argv, value)) {
        usage();
        return 2;
      }
      options.diff_threshold_percent = std::atof(value.c_str());
      continue;
    }
    std::cerr << "unknown option: " << arg << "\n";
    usage();
    return 2;
  }

  return cppautogui::run_launch_tests(options);
}
