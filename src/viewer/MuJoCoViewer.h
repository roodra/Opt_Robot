#pragma once

#include <memory>
#include <vector>

class MuJoCoEngine;

// Internal struct holding mjv/mjr state — defined only in MuJoCoViewer.cpp.
// Keeps MuJoCo and GLFW headers out of this header entirely.
struct MjvState;

// Replays a torque sequence on a MuJoCoEngine in a GLFW window.
// Engine-specific by design — rendering requires direct access to mjModel/mjData.
class MuJoCoViewer {
public:
    explicit MuJoCoViewer(MuJoCoEngine& engine);
    ~MuJoCoViewer();

    // Play back a sequence of control vectors at real-time speed.
    // controls[i] is applied at step i; window stays open until closed.
    void replay(const std::vector<std::vector<double>>& controls, double dt);

private:
    bool init();
    void render();

    MuJoCoEngine&           m_engine;
    void*                   m_window = nullptr;  // GLFWwindow*
    std::unique_ptr<MjvState> m_mjv;
};
