#pragma once

#include "spdlog/spdlog.h"
#include "imgui.h"

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>

static void HelpMarker(const char* desc);

struct tmpstringstream {
    std::ostringstream ss;
    template<class T> tmpstringstream & operator << (const T & val) { ss << val; return *this; }
    operator std::string() const { return ss.str(); }
};

inline std::string pretty_time(std::chrono::nanoseconds d) {
    auto hhh = std::chrono::duration_cast<std::chrono::hours>(d);
    d -= hhh;
    auto mm = std::chrono::duration_cast<std::chrono::minutes>(d);
    d -= mm;
    auto ss = std::chrono::duration_cast<std::chrono::seconds>(d);
    d -= ss;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(d);

    std::ostringstream stream;
    stream << std::setfill('0') << std::setw(3) << hhh.count() << ':' <<
           std::setfill('0') << std::setw(2) << mm.count() << ':' <<
           std::setfill('0') << std::setw(2) << ss.count() << '.' <<
           std::setfill('0') << std::setw(3) << ms.count();
    return stream.str();
}
void setup_logger();

void cleanup_logger();

ImVec4 from_rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a, bool consistent_color = false);

std::vector<std::string> get_json_files(const std::string& directoryPath);