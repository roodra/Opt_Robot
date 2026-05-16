#include "MuJoCoViewer.h"

#include "../physics/mujoco/MuJoCoEngine.h"

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <chrono>
#include <iostream>
#include <thread>

// Holds all mjv/mjr state — allocated on the heap so mujoco.h stays out of the header.
struct MjvState {
    mjvCamera  cam;
    mjvOption  opt;
    mjvScene   scn;
    mjrContext con;
};

// --- GLFW callbacks ---

static void key_callback(GLFWwindow* window, int key, int /*scancode*/,
                         int action, int /*mods*/) {
    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

static void scroll_callback(GLFWwindow* window, double /*xoffset*/, double yoffset) {
    auto* cam = static_cast<mjvCamera*>(glfwGetWindowUserPointer(window));
    cam->distance = mjMAX(0.5, cam->distance - yoffset * 0.2);
}

// --- MuJoCoViewer ---

MuJoCoViewer::MuJoCoViewer(MuJoCoEngine& engine)
    : m_engine(engine)
{}

MuJoCoViewer::~MuJoCoViewer() {
    if (m_mjv) {
        mjv_freeScene(&m_mjv->scn);
        mjr_freeContext(&m_mjv->con);
    }
    if (m_window)
        glfwDestroyWindow(static_cast<GLFWwindow*>(m_window));
    glfwTerminate();
}

bool MuJoCoViewer::init() {
    if (!glfwInit()) {
        std::cerr << "GLFW init failed.\n";
        return false;
    }

    GLFWwindow* win = glfwCreateWindow(1200, 900, "Pendulum Swing-Up", nullptr, nullptr);
    if (!win) {
        std::cerr << "GLFW window creation failed.\n";
        glfwTerminate();
        return false;
    }
    m_window = win;

    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    m_mjv = std::make_unique<MjvState>();
    auto* m = static_cast<mjModel*>(m_engine.getModel());

    mjv_defaultCamera(&m_mjv->cam);
    mjv_defaultOption(&m_mjv->opt);
    mjv_makeScene(m, &m_mjv->scn, 2000);
    mjr_makeContext(m, &m_mjv->con, mjFONTSCALE_150);

    // Position camera to see the full pendulum swing
    m_mjv->cam.distance  = 3.5;
    m_mjv->cam.azimuth   = 90.0;
    m_mjv->cam.elevation = -20.0;

    // Pass camera pointer to scroll callback via user pointer
    glfwSetWindowUserPointer(win, &m_mjv->cam);
    glfwSetKeyCallback(win, key_callback);
    glfwSetScrollCallback(win, scroll_callback);

    return true;
}

void MuJoCoViewer::render() {
    auto* win = static_cast<GLFWwindow*>(m_window);
    auto* m   = static_cast<mjModel*>(m_engine.getModel());
    auto* d   = static_cast<mjData*>(m_engine.getData());

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(win, &viewport.width, &viewport.height);

    mjv_updateScene(m, d, &m_mjv->opt, nullptr, &m_mjv->cam, mjCAT_ALL, &m_mjv->scn);
    mjr_render(viewport, &m_mjv->scn, &m_mjv->con);

    glfwSwapBuffers(win);
    glfwPollEvents();
}

void MuJoCoViewer::replay(const std::vector<std::vector<double>>& controls, double dt) {
    if (!init()) return;

    auto* win = static_cast<GLFWwindow*>(m_window);

    using clock    = std::chrono::steady_clock;
    using duration = std::chrono::duration<double>;

    // Loop: play trajectory once, then hold final frame until window is closed
    std::size_t step = 0;
    m_engine.setState({0.0}, {0.0});

    while (!glfwWindowShouldClose(win)) {
        auto t0 = clock::now();

        if (step < controls.size()) {
            m_engine.setControl(controls[step]);
            m_engine.step(dt);
            ++step;
        }

        render();

        // Sleep remainder of the timestep to match real-time playback
        auto elapsed = clock::now() - t0;
        auto remaining = duration(dt) - elapsed;
        if (remaining.count() > 0)
            std::this_thread::sleep_for(remaining);
    }
}
