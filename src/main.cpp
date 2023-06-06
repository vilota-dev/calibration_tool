#include "utils/utils.h"
#include "gui/gui.h"

int main(int argc, char *argv[]) {
    setup_logger();
    run_gui();
    cleanup_logger();

    return 0;
}
