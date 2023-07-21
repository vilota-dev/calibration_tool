#pragma once

#include <imgui.h>

#include <string>

class View {
public:
    explicit View(std::string name);
    virtual ~View() = default;

    virtual void draw_content() = 0;
    [[nodiscard]] inline const std::string& get_name() const { return this->name; }
private:
    std::string name;
};