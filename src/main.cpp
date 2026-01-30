#include <iostream>
#include <mujoco/mujoco.h>
#include "include/Qlearning.h"
#include "include/Environment.h"
#include "include/Viewer.h"

int main() {
    std::cout << "MuJoCo version: "
              << mj_versionString()
              << std::endl;
    return 0;
}
