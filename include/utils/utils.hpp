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

inline std::string pretty_time(std::chrono::nanoseconds d);

void setup_logger();

void cleanup_logger();

ImVec4 from_rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a, bool consistent_color = false);