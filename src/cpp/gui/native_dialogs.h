#pragma once

#include <string>
#include <vector>

namespace hesim3d {

class NativeDialogs {
 public:
  // Prompt user to select an existing file
  static std::string open_file(const std::string& title, const std::string& default_path,
                               const std::vector<std::string>& filters = {"All Files", "*"});

  // Prompt user to choose a save destination file
  static std::string save_file(const std::string& title, const std::string& default_path,
                               const std::vector<std::string>& filters = {"All Files", "*"});

  // Prompt user to select a folder
  static std::string select_folder(const std::string& title, const std::string& default_path);

  // Open target folder in system file explorer (Linux xdg-open, macOS open, Windows explorer)
  static bool open_in_system_explorer(const std::string& folder_path);
};

}  // namespace hesim3d
