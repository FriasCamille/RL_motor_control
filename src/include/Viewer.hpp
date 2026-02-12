#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#define ERROR ("\x1b[1;31m")
#define RESET_FORMAT ("\x1b[1;0m")

class Viewer
{
    private:
        GLFWwindow* window;
        mjvCamera cam;
        mjvOption opt;
        mjvScene scn;
        mjrContext con;

        mjModel* m;
        mjData* d;
    public:

        Viewer(mjModel* model, mjData* data);
        ~Viewer();
        void render();
        void update();
        bool should_close();
};