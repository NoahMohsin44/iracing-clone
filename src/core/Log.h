#pragma once
#include <string>

namespace apex {

void logInit(const std::string& filePath);
void logShutdown();
void logMessage(const char* level, const char* fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;

}  // namespace apex

#define LOGI(...) ::apex::logMessage("info", __VA_ARGS__)
#define LOGW(...) ::apex::logMessage("warn", __VA_ARGS__)
#define LOGE(...) ::apex::logMessage("error", __VA_ARGS__)
