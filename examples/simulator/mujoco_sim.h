#ifndef EXAMPLES_SIMULATOR_MUJOCO_SIM_HPP
#define EXAMPLES_SIMULATOR_MUJOCO_SIM_HPP

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>

using namespace std::placeholders;

class MujocoSimulator {
   public:
    ~MujocoSimulator() {
        // free visualization storage
        mjv_freeScene(&scn_);
        mjr_freeContext(&con_);

        // free MuJoCo model and data
        mj_deleteData(d_);
        mj_deleteModel(m_);

        // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
#endif
    }

    static MujocoSimulator& getInstance()  // Singleton is accessed via getInstance()
    {
        static MujocoSimulator instance;  // lazy singleton, instantiated on first use
        return instance;
    }

    int Init();
    int LoadModel(const char* xml_file);
    int ForwardStep();
    int ApplyControl(double* ctrl, int nu);
    int UpdateSceneAndRender();

    int WindowShouldClose() {
        return glfwWindowShouldClose(window_);
    }

    int GetModelNq() { return m_->nq; };
    int GetModelNv() { return m_->nv; };

    /**
     * @brief Returns the pointer to the qvel data array for the configuration
     * of the model (nq x 1)
     *
     * @return const mjtNum*
     */
    const mjtNum* GetModelConfiguration() { return d_->qpos; }

    /**
     * @brief Returns the pointer to the qvel data array for the velocity
     * of the model (nv x 1)
     *
     * @return const mjtNum*
     */
    const mjtNum* GetModelVelocity() { return d_->qvel; }

    /**
     * @brief Returns the pointer to the qacc data array for the acceeleration
     * of the model (nv x 1)
     *
     * @return const mjtNum*
     */
    const mjtNum* GetModelAcceleration() { return d_->qacc; }

    /**
     * @brief Returns the current time within the simulation.
     *
     * @return const double
     */
    const double GetSimulatorTime() { return d_->time; };

   private:
    // Determines if the system has been iniitalised
    bool is_init_ = false;

    mjtNum t_;
    mjtNum t_frame_start_;

    // GLFW window
    GLFWwindow* window_;

    // MuJoCo data structures
    mjModel* m_;      // MuJoCo model
    mjData* d_;       // MuJoCo data
    mjvCamera cam_;   // abstract camera
    mjvOption opt_;   // visualization options
    mjvScene scn_;    // abstract scene
    mjrContext con_;  // custom GPU context

    // mouse interaction
    bool button_left_ = false;
    bool button_middle_ = false;
    bool button_right_ = false;
    double lastx_ = 0;
    double lasty_ = 0;

    // Private constructor to create only one instance
    MujocoSimulator(void) {}

    MujocoSimulator(MujocoSimulator const&);  // prevent copies
    void operator=(MujocoSimulator const&);   // prevent assignments

    static void MouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
        // here we access the instance via the singleton pattern and forward the callback to the instance method
        getInstance().MouseButtonCallbackImpl(window, button, act, mods);
    }

    static void KeyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods) {
        // here we access the instance via the singleton pattern and forward the callback to the instance method
        getInstance().KeyboardCallbackImpl(window, key, scancode, act, mods);
    }

    static void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
        // here we access the instance via the singleton pattern and forward the callback to the instance method
        getInstance().MouseMoveCallbackImpl(window, xpos, ypos);
    }

    static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
        // here we access the instance via the singleton pattern and forward the callback to the instance method
        getInstance().ScrollCallbackImpl(window, xoffset, yoffset);
    }

    void KeyboardCallbackImpl(GLFWwindow* window, int key, int scancode, int act, int mods);
    void MouseButtonCallbackImpl(GLFWwindow* window, int button, int act, int mods);
    void MouseMoveCallbackImpl(GLFWwindow* window, double xpos, double ypos);
    void ScrollCallbackImpl(GLFWwindow* window, double xoffset, double yoffset);
};

#endif /* EXAMPLES_SIMULATOR_MUJOCO_SIM_HPP */
