#include "include/Viewer.hpp"
#include <iostream>
#include <string>

Viewer::Viewer(mjModel* model, mjData* data)
        : m(model), d(data)
{
    
    if (!glfwInit()) 
    {
        throw std::runtime_error((std::string) ERROR+"No se pudo inicializar GLFW/n" + RESET_FORMAT);
    }

    window = glfwCreateWindow(1200, 900, "MuJoCo Viewer", NULL, NULL);
    if (!window) 
    {
        glfwTerminate();
        throw std::runtime_error((std::string) ERROR+"No se pudo Abrir una ventana de GLFW/n" + RESET_FORMAT);
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); 


    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);


    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);


    cam.lookat[0] = 0;
    cam.lookat[1] = 0;
    cam.lookat[2] = 1.0;
    cam.distance = 3.0;
    cam.azimuth = 90;
    cam.elevation = -20;
}

Viewer::~Viewer() 
{
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
}

void Viewer::render() {
        
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

void Viewer::update()
{
    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
}

bool Viewer::should_close() 
{
    return glfwWindowShouldClose(window);
}