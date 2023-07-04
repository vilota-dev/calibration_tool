#include "gui/gui.h"
#include "utils/utils.hpp"

int main(int argc, char *argv[]) {
    setup_logger();
    run_gui();
    cleanup_logger();

    return 0;
}
