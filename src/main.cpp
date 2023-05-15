#include "utils/utils.h"
#include "gui/gui.h"

int main(int, char **) {
    // Initialize logger
    setup_logger();

    run_gui();

    // Cleanup logger
    cleanup_logger();

    return 0;
}
