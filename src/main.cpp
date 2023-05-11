#include "utils/utils.hpp"
#include "gui/gui.hpp"


int main(int, char **) {
    // Initialize logger
    setup_logger();

    run_gui();

    // Cleanup logger
    cleanup_logger();

    return 0;
}
