#include "utils/utils.hpp"
#include "ui/window.hpp"

int main(int argc, char *argv[]) {
    setup_logger();

    using namespace vk;
    Window window;
    window.loop();

    cleanup_logger();
}
