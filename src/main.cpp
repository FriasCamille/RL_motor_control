#include <iostream>
#include <mujoco/mujoco.h>

int main() {
    std::cout << "MuJoCo version: "
              << mj_versionString()
              << std::endl;
    return 0;
}
