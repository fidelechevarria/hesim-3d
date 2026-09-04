#include "native_dialogs.h"
#include "portable-file-dialogs.h"
#include <iostream>
#include <cstdlib>

namespace hesim3d {

std::string NativeDialogs::open_file(
    const std::string& title,
    const std::string& default_path,
    const std::vector<std::string>& filters
) {
    if (!pfd::settings::available()) {
        std::cerr << "[NativeDialogs] No desktop dialog backend (zenity/kdialog) available." << std::endl;
        return "";
    }

    try {
        auto sel = pfd::open_file(title, default_path, filters, pfd::opt::none);
        auto res = sel.result();
        if (!res.empty()) {
            return res[0];
        }
    } catch (const std::exception& e) {
        std::cerr << "[NativeDialogs] open_file error: " << e.what() << std::endl;
    }
    return "";
}

std::string NativeDialogs::save_file(
    const std::string& title,
    const std::string& default_path,
    const std::vector<std::string>& filters
) {
    if (!pfd::settings::available()) {
        std::cerr << "[NativeDialogs] No desktop dialog backend (zenity/kdialog) available." << std::endl;
        return "";
    }

    try {
        auto sel = pfd::save_file(title, default_path, filters, pfd::opt::none);
        return sel.result();
    } catch (const std::exception& e) {
        std::cerr << "[NativeDialogs] save_file error: " << e.what() << std::endl;
    }
    return "";
}

std::string NativeDialogs::select_folder(
    const std::string& title,
    const std::string& default_path
) {
    if (!pfd::settings::available()) {
        std::cerr << "[NativeDialogs] No desktop dialog backend (zenity/kdialog) available." << std::endl;
        return "";
    }

    try {
        auto sel = pfd::select_folder(title, default_path, pfd::opt::none);
        return sel.result();
    } catch (const std::exception& e) {
        std::cerr << "[NativeDialogs] select_folder error: " << e.what() << std::endl;
    }
    return "";
}

bool NativeDialogs::open_in_system_explorer(const std::string& folder_path) {
    if (folder_path.empty()) return false;

#if defined(__APPLE__)
    std::string cmd = "open \"" + folder_path + "\" &";
#elif defined(_WIN32)
    std::string cmd = "explorer \"" + folder_path + "\"";
#else
    std::string cmd = "xdg-open \"" + folder_path + "\" >/dev/null 2>&1 &";
#endif

    int ret = std::system(cmd.c_str());
    return (ret == 0);
}

} // namespace hesim3d
