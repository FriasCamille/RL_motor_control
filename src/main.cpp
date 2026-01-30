#include <iostream>
#include <mujoco/mujoco.h>
#include "include/Q_learning.h"
#include "include/environment.h"
#include "include/viewer.h"

int main() {
    std::cout << "MuJoCo version: "
              << mj_versionString()
              << std::endl;
    return 0;
}
