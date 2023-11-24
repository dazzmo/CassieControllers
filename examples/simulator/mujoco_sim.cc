#include "mujoco_sim.h"

int MujocoSimulator::LoadModel(const char* xml_file) {
    // load and compile model
    char error[1000] = "Could not load binary model";
    if (std::strlen(xml_file) > 4 &&
        !std::strcmp(xml_file + std::strlen(xml_file) - 4, ".mjb")) {
        m_ = mj_loadModel(xml_file, 0);
    } else {
        m_ = mj_loadXML(xml_file, 0, error, 1000);
    }
    if (!m_) {
        mju_error_s("Load model error: %s", error);
        return 1;
    }
    // Create data from model
    d_ = mj_makeData(m_);
    return 0;
}

int MujocoSimulator::Init() {
    // init GLFW
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    window_ = glfwCreateWindow(1200, 900, "mujoco simulator", NULL, NULL);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    // create scene and context
    mjv_makeScene(m_, &scn_, 2000);
    mjr_makeContext(m_, &con_, mjFONTSCALE_150);

    // install GLFW mouse and KeyboardCallbackImpl callbacks
    glfwSetKeyCallback(window_, MujocoSimulator::KeyboardCallback);
    glfwSetCursorPosCallback(window_, MujocoSimulator::MouseMoveCallback);
    glfwSetMouseButtonCallback(window_, MujocoSimulator::MouseButtonCallback);
    glfwSetScrollCallback(window_, MujocoSimulator::ScrollCallback);

    return 0;
}

int MujocoSimulator::ForwardStep() {
    if (paused_) {
        mj_forward(m_, d_);
    } else {
        mj_step(m_, d_);
    }
    return 0;
}

int MujocoSimulator::ApplyControl(const double* ctrl, int nu) {
    mju_copy(d_->ctrl, ctrl, nu);
    return 0;
}

int MujocoSimulator::UpdateSceneAndRender() {
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    // update scene and render
    mjv_updateScene(m_, d_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport, &scn_, &con_);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    return 0;
}

/**
 * @brief Keyboard callback implementation
 *
 * @param window
 * @param key
 * @param scancode
 * @param act
 * @param mods
 */
void MujocoSimulator::KeyboardCallbackImpl(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        ResetModel();
        mj_forward(m_, d_);
    }
    // space: pause/unpause simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
        paused_ = !paused_;
    }
    // quit window
    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

// mouse button callback
void MujocoSimulator::MouseButtonCallbackImpl(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx_, &lasty_);
}

// mouse move callback
void MujocoSimulator::MouseMoveCallbackImpl(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left_ && !button_middle_ && !button_right_) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx_;
    double dy = ypos - lasty_;
    lastx_ = xpos;
    lasty_ = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right_) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left_) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(m_, action, dx / height, dy / height, &scn_, &cam_);
}

// scroll callback
void MujocoSimulator::ScrollCallbackImpl(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn_, &cam_);
}