#include "core/Paths.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#include <windows.h>
#endif

namespace fs = std::filesystem;

namespace apex::paths {

namespace {
std::string g_assets;
std::string g_user;

fs::path executableDir() {
#ifdef _WIN32
    wchar_t buf[MAX_PATH];
    DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    if (n > 0) return fs::path(std::wstring(buf, n)).parent_path();
#else
    std::error_code ec;
    fs::path p = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) return p.parent_path();
#endif
    return fs::current_path();
}

bool looksLikeAssets(const fs::path& p) {
    std::error_code ec;
    return fs::is_directory(p / "shaders", ec) && fs::is_directory(p / "tracks", ec);
}

std::string withSlash(fs::path p) {
    std::string s = p.lexically_normal().generic_string();
    if (s.empty() || s.back() != '/') s += '/';
    return s;
}
}  // namespace

void init() {
    std::vector<fs::path> candidates;
    for (fs::path dir = executableDir(); !dir.empty(); dir = dir.parent_path()) {
        candidates.push_back(dir / "assets");
        if (dir == dir.parent_path()) break;
    }
    candidates.push_back(fs::current_path() / "assets");
#ifdef APEX_SOURCE_ASSETS
    candidates.push_back(fs::path(APEX_SOURCE_ASSETS));
#endif
    for (auto& c : candidates) {
        if (looksLikeAssets(c)) {
            g_assets = withSlash(c);
            break;
        }
    }
    if (g_assets.empty()) g_assets = "assets/";

    fs::path user;
#ifdef _WIN32
    if (const char* appdata = std::getenv("APPDATA")) user = fs::path(appdata) / "ApexSim";
#else
    if (const char* xdg = std::getenv("XDG_CONFIG_HOME")) user = fs::path(xdg) / "apexsim";
    else if (const char* home = std::getenv("HOME")) user = fs::path(home) / ".config" / "apexsim";
#endif
    if (user.empty()) user = fs::current_path() / "userdata";
    std::error_code ec;
    fs::create_directories(user, ec);
    fs::create_directories(user / "screenshots", ec);
    g_user = withSlash(user);
}

const std::string& assetRoot() { return g_assets; }
std::string asset(const std::string& relative) { return g_assets + relative; }
const std::string& userDir() { return g_user; }

bool readTextFile(const std::string& path, std::string& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

bool readBinaryFile(const std::string& path, std::vector<unsigned char>& out) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return false;
    std::streamsize size = f.tellg();
    f.seekg(0);
    out.resize(static_cast<size_t>(size));
    return size == 0 || static_cast<bool>(f.read(reinterpret_cast<char*>(out.data()), size));
}

}  // namespace apex::paths
