#include "utils/utils.h"
#include "gui/gui.h"

int main(int, char **) {
    setup_logger();
    run_gui();
    cleanup_logger();

    return 0;
}
