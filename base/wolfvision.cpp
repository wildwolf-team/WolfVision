#include "wolfvision.hpp"

int main() {
    fmt::print("[wolfvision] WolfVision built on g++ version: {}\n", __VERSION__);
    fmt::print("[wolfvision] WolfVision config file path: {}\n", CONFIG_FILE_PATH);

    return 0;
}
