#include "core/Log.h"

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace apex {

namespace {
std::mutex g_logMutex;
FILE* g_logFile = nullptr;
const auto g_start = std::chrono::steady_clock::now();
}  // namespace

void logInit(const std::string& filePath) {
    std::lock_guard<std::mutex> lock(g_logMutex);
    if (g_logFile) fclose(g_logFile);
    g_logFile = fopen(filePath.c_str(), "w");
}

void logShutdown() {
    std::lock_guard<std::mutex> lock(g_logMutex);
    if (g_logFile) fclose(g_logFile);
    g_logFile = nullptr;
}

void logMessage(const char* level, const char* fmt, ...) {
    char msg[2048];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - g_start).count();
    std::lock_guard<std::mutex> lock(g_logMutex);
    fprintf(stdout, "[%8.3f] %-5s %s\n", t, level, msg);
    fflush(stdout);
    if (g_logFile) {
        fprintf(g_logFile, "[%8.3f] %-5s %s\n", t, level, msg);
        fflush(g_logFile);
    }
}

}  // namespace apex
