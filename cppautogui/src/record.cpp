#include "cppautogui.hpp"

#include <iostream>
#include <string>

namespace {

void usage()
{
  std::cout << "record_cppautogui -i|--itest <test_name> [-d|--debug] [-r|--retina]\n";
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
  cppautogui::RecordOptions options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      usage();
      return 0;
    }
    if (arg == "-i" || arg == "--itest" || arg == "-itest") {
      if (!read_value(i, argc, argv, options.app_name)) {
        usage();
        return 2;
      }
      continue;
    }
    if (arg == "-d" || arg == "--debug") {
      options.debug = true;
      continue;
    }
    if (arg == "-r" || arg == "--retina") {
      options.retina = true;
      continue;
    }
    std::cerr << "unknown option: " << arg << "\n";
    usage();
    return 2;
  }

  return cppautogui::run_record(options);
}
