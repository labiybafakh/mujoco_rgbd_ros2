#include "mujoco_rgbd_camera.hpp"
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <OpenGL/gl.h>
#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>
#include <signal.h>
#include <atomic>
#include <algorithm>

std::atomic<bool> running(true);

void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ", shutting down..." << std::endl;
    running = false;
}

int main(int argc, char** argv) {
    std::cout << "Starting MuJoCo RGBD application..." << std::endl;
    std::cout.flush();
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "Signal handlers set up" << std::endl;
    std::cout.flush();

    std::string model_file = "../config/camera_environment.xml";
    std::string camera_name = "camera";
    
    if (argc > 1) {
        model_file = argv[1];
    }
    if (argc > 2) {
        camera_name = argv[2];
    }

    std::cout << "Loading MuJoCo model: " << model_file << std::endl;
    std::cout << "Using camera: " << camera_name << std::endl;
    std::cout.flush();

    // Load MuJoCo model
    std::cout << "About to load MuJoCo model..." << std::endl;
    std::cout.flush();
    
    char error[10000];
    std::memset(error, 0, sizeof(error));
    mjModel* model = mj_loadXML(model_file.c_str(), nullptr, error, sizeof(error));
    
    if (!model) {
        std::cerr << "Error: " << error << std::endl;
        return -1;
    }

    std::cout << "MuJoCo model loaded successfully" << std::endl;
    std::cout.flush();
    
    mjData* data = mj_makeData(model);
    std::cout << "MuJoCo data created successfully" << std::endl;
    std::cout.flush();

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        mj_deleteData(data);
        mj_deleteModel(model);
        return -1;
    }

    // Create visible context for rendering (use default OpenGL settings)
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    GLFWwindow* window = glfwCreateWindow(1024, 768, "MuJoCo Simulation - ESC to quit", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        mj_deleteData(data);
        mj_deleteModel(model);
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    
    // Make sure window is focused and visible
    glfwShowWindow(window);
    glfwFocusWindow(window);

    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "OpenGL Renderer: " << glGetString(GL_RENDERER) << std::endl;

    // Find camera (optional for visualization)
    int camera_id = -1;
    if (model->ncam > 0) {
        camera_id = mj_name2id(model, mjOBJ_CAMERA, camera_name.c_str());
        if (camera_id == -1) {
            std::cout << "Camera '" << camera_name << "' not found, using first available camera" << std::endl;
            camera_id = 0; // Use first camera
        } else {
            std::cout << "Found camera '" << camera_name << "' with ID: " << camera_id << std::endl;
        }
    } else {
        std::cout << "No cameras in model, visualization only" << std::endl;
    }
    std::cout << "Model has " << model->nbody << " bodies, " << model->ngeom << " geometries" << std::endl;
    
    // Debug: print some body and geom info
    for (int i = 0; i < model->nbody && i < 5; i++) {
        const char* name = mj_id2name(model, mjOBJ_BODY, i);
        std::cout << "  Body " << i << ": " << (name ? name : "unnamed") << std::endl;
    }
    for (int i = 0; i < model->ngeom && i < 5; i++) {
        const char* name = mj_id2name(model, mjOBJ_GEOM, i);
        std::cout << "  Geom " << i << ": " << (name ? name : "unnamed") << std::endl;
    }

    // Initialize rendering context
    mjrContext context;
    mjr_defaultContext(&context);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    // Initialize visualization structures
    mjvOption opt;
    mjvScene scn;
    mjvCamera cam;
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjv_defaultCamera(&cam);
    mjv_makeScene(model, &scn, 2000);
    
    // Check if scene was created properly
    std::cout << "Scene allocated with max capacity: " << scn.maxgeom << " geoms" << std::endl;
    
    // Enable more visualization options
    opt.flags[mjVIS_TRANSPARENT] = 1;  // Show transparent objects
    opt.flags[mjVIS_CONTACTPOINT] = 0; // Disable contact points
    opt.flags[mjVIS_CONTACTFORCE] = 0; // Disable contact forces
    opt.flags[mjVIS_STATIC] = 1;       // Show static bodies
    opt.flags[mjVIS_CONVEXHULL] = 0;   // Disable convex hulls
    opt.flags[mjVIS_TEXTURE] = 1;      // Enable textures (for checkered floor)
    
    // Enable geom groups 
    for (int i = 0; i < mjNGROUP; i++) {
        opt.geomgroup[i] = 1;  // Enable all geom groups
    }

    // Set camera position to match original implementation
    cam.azimuth = 45.0;
    cam.elevation = -25.0;
    cam.distance = 10.0;
    cam.lookat[0] = 2.5;
    cam.lookat[1] = 0.5;
    cam.lookat[2] = 1.0;

    std::cout << "Visualization initialized. Camera distance: " << cam.distance << std::endl;
    
    // Add mouse interaction
    glfwSetWindowUserPointer(window, &cam);
    glfwSetCursorPosCallback(window, [](GLFWwindow* win, double x, double y) {
        static double lastx = x, lasty = y;
        static bool button_left = false, button_right = false;
        
        int state_left = glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT);
        int state_right = glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT);
        
        button_left = (state_left == GLFW_PRESS);
        button_right = (state_right == GLFW_PRESS);
        
        if (button_left || button_right) {
            mjvCamera* cam = static_cast<mjvCamera*>(glfwGetWindowUserPointer(win));
            
            double dx = x - lastx;
            double dy = y - lasty;
            
            if (button_right) {
                // Right drag: rotate
                cam->azimuth += dx * 0.5;
                cam->elevation -= dy * 0.5;
                // Clamp elevation
                if (cam->elevation < -90) cam->elevation = -90;
                if (cam->elevation > 90) cam->elevation = 90;
            } else if (button_left) {
                // Left drag: zoom
                cam->distance *= (1.0 + 0.01 * dy);
                if (cam->distance < 0.1) cam->distance = 0.1;
                if (cam->distance > 100) cam->distance = 100;
            }
        }
        
        lastx = x;
        lasty = y;
    });
    
    std::cout << "Mouse controls enabled: Right-click+drag to rotate, Left-click+drag to zoom" << std::endl;
    
    // Add keyboard controls
    glfwSetKeyCallback(window, [](GLFWwindow* win, int key, int scancode, int action, int mods) {
        mjvCamera* cam = static_cast<mjvCamera*>(glfwGetWindowUserPointer(win));
        
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            glfwSetWindowShouldClose(win, GLFW_TRUE);
        }
        else if (key == GLFW_KEY_R && action == GLFW_PRESS) {
            // Reset camera to original position
            cam->azimuth = 90.0;
            cam->elevation = -20.0;
            cam->distance = 5.0;
            cam->lookat[0] = 0.0;
            cam->lookat[1] = 0.0;
            cam->lookat[2] = 1.0;
            std::cout << "Camera reset to original position" << std::endl;
        }
        else if (key == GLFW_KEY_1 && action == GLFW_PRESS) {
            // View 1: Side view of scene
            cam->azimuth = 90.0;
            cam->elevation = -15.0;
            cam->distance = 8.0;
            cam->lookat[0] = 2.0;
            cam->lookat[1] = 0.0;
            cam->lookat[2] = 1.0;
            std::cout << "Camera view 1: Side view" << std::endl;
        }
        else if (key == GLFW_KEY_2 && action == GLFW_PRESS) {
            // View 2: Diagonal overview from opposite side
            cam->azimuth = 315.0;  // Opposite side (135 + 180 = 315)
            cam->elevation = -25.0;
            cam->distance = 10.0;
            cam->lookat[0] = 2.5;
            cam->lookat[1] = 0.5;
            cam->lookat[2] = 1.0;
            std::cout << "Camera view 2: Diagonal overview from opposite side" << std::endl;
        }
    });
    
    std::cout << "Keyboard controls: ESC=quit, R=reset camera, 1/2=preset views" << std::endl;

    // Temporarily disable RGBD camera to test basic visualization
    /*
    MujocoRGBDCamera rgbd_camera;
    if (!rgbd_camera.initialize(model, camera_id)) {
        std::cerr << "Failed to initialize RGBD camera" << std::endl;
        mjr_freeContext(&context);
        glfwDestroyWindow(window);
        glfwTerminate();
        mj_deleteData(data);
        mj_deleteModel(model);
        return -1;
    }
    std::cout << "RGBD camera initialized successfully" << std::endl;
    */
    
    std::cout << "Starting main loop..." << std::endl;
    
    int frame_count = 0;
    auto last_time = std::chrono::high_resolution_clock::now();
    
    // Main loop
    while (running && !glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Run simulation step
        mj_step(model, data);

        // Update scene - need to pass perturbation and camera properly
        mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);

        // Render
        mjrRect viewport;
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        viewport.left = 0;
        viewport.bottom = 0;
        
        // Set OpenGL viewport
        glViewport(0, 0, viewport.width, viewport.height);
        
        // Enable depth testing
        glEnable(GL_DEPTH_TEST);
        
        // Clear background 
        glClearColor(0.2f, 0.3f, 0.4f, 1.0f);  // Sky blue background
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        mjr_render(viewport, &scn, &context);
        
        // Check for OpenGL errors
        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            std::cerr << "OpenGL error: " << error << std::endl;
        }
        
        // Debug: print every 100 frames for easier adjustment
        if (frame_count % 100 == 0) {
            std::cout << "Rendering frame " << frame_count << " viewport: " << viewport.width << "x" << viewport.height << std::endl;
            std::cout << "Camera position - azimuth: " << cam.azimuth << ", elevation: " << cam.elevation << ", distance: " << cam.distance << std::endl;
            std::cout << "Camera lookat: [" << cam.lookat[0] << ", " << cam.lookat[1] << ", " << cam.lookat[2] << "]" << std::endl;
            std::cout << "Scene objects: " << scn.ngeom << " geoms" << std::endl;
            // Print first few geom positions and types
            for (int i = 0; i < std::min(5, scn.ngeom); i++) {
                std::cout << "  Geom " << i << " pos: [" << scn.geoms[i].pos[0] << ", " << scn.geoms[i].pos[1] << ", " << scn.geoms[i].pos[2] << "] type: " << scn.geoms[i].type << std::endl;
            }
        }

        // Temporarily disabled RGBD capture
        /*
        if (!rgbd_camera.capture(model, data, &context)) {
            std::cerr << "Warning: Failed to capture RGBD data" << std::endl;
        } else {
        */
            frame_count++;
            
            // Print stats every 100 frames
            if (frame_count % 100 == 0) {
                auto current_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - last_time);
                double fps = 100.0 * 1000.0 / duration.count();
                
                std::cout << "Frame " << frame_count << " - FPS: " << fps << std::endl;
                std::cout << "Basic visualization running" << std::endl;
                
                last_time = current_time;
            }
        /*
        }
        */

        // Swap buffers
        glfwSwapBuffers(window);

        // Run at ~30 FPS
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // Cleanup
    std::cout << "Cleaning up..." << std::endl;
    mjv_freeScene(&scn);
    mjr_freeContext(&context);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(data);
    mj_deleteModel(model);

    std::cout << "Shutdown complete. Total frames processed: " << frame_count << std::endl;
    return 0;
}