#include "utils/utils.hpp"

static void HelpMarker(const char* desc) {
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort) && ImGui::BeginTooltip())
    {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

inline std::string pretty_time(std::chrono::nanoseconds d)
{
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

void setup_logger() {
    try {
        // Customize msg format for all loggers
        spdlog::set_pattern("[%D %H:%M:%S] [%^%L%$] [thread %t] %v");
        // Set global log level to info
        spdlog::set_level(spdlog::level::trace);

        // Flush all *registered* loggers using a worker thread every 3 seconds.
        // note: registered loggers *must* be thread safe for this to work correctly!
        spdlog::flush_every(std::chrono::seconds(3));
    }
        // Exceptions will only be thrown upon failed logger or sink construction (not during logging).
    catch (const spdlog::spdlog_ex &ex) {
        std::printf("Log initialization failed: %s\n", ex.what());
        exit(1);
    }
}

void cleanup_logger() {
    // Apply some function on all registered loggers
    spdlog::apply_all([&](std::shared_ptr<spdlog::logger> l) { l->debug("App exit"); });
    // Release all spdlog resources, and drop all loggers in the registry
    spdlog::shutdown();
}

ImVec4 from_rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a, bool consistent_color) {
    auto res = ImVec4(r / (float)255, g / (float)255, b / (float)255, a / (float)255);
    return res;
}
