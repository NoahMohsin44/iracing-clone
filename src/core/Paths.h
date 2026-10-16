#pragma once
#include <string>
#include <vector>

namespace apex::paths {

// Locates the assets folder (next to the executable, in a parent folder, or the source tree)
// and the per-user folder used for settings, logs and screenshots.
void init();

const std::string& assetRoot();   // ends with '/'
std::string asset(const std::string& relative);
const std::string& userDir();     // ends with '/'

bool readTextFile(const std::string& path, std::string& out);
bool readBinaryFile(const std::string& path, std::vector<unsigned char>& out);

}  // namespace apex::paths
