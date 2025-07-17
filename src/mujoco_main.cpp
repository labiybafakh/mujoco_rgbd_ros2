#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

// Simple 3D vector and matrix structures
struct Vec3 {
    double x, y, z;
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

struct Vec6 {
    double x, y, z, r, g, b;
    Vec6(double x = 0, double y = 0, double z = 0, double r = 0, double g = 0, double b = 0) 
        : x(x), y(y), z(z), r(r), g(g), b(b) {}
};

struct Mat3 {
    double data[9];
    Mat3() { for(int i = 0; i < 9; i++) data[i] = 0; }
    double& operator()(int i, int j) { return data[i*3 + j]; }
    const double& operator()(int i, int j) const { return data[i*3 + j]; }
};

struct Mat4 {
    double data[16];
    Mat4() { 
        for(int i = 0; i < 16; i++) data[i] = 0;
        // Initialize as identity
        data[0] = data[5] = data[10] = data[15] = 1.0;
    }
    double& operator()(int i, int j) { return data[i*4 + j]; }
    const double& operator()(int i, int j) const { return data[i*4 + j]; }
};

class MuJoCoRGBDCapture {
private:
    mjModel* model;
    mjData* data;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    
    int width, height;
    int camera_id;
    double fps;
    
    Mat3 intrinsic_matrix;
    Mat4 extrinsic_matrix;
    
    std::vector<unsigned char> rgb_buffer;
    std::vector<float> depth_buffer;
    
    GLFWwindow* window;

public:
    MuJoCoRGBDCapture(const char* model_path = "../model.xml", int w = 640, int h = 480, double frame_rate = 30.0) 
        : width(w), height(h), fps(frame_rate), window(nullptr) {
        
        // Initialize GLFW
        if (!glfwInit()) {
            throw std::runtime_error("Failed to initialize GLFW");
        }
        
        // Create a visible window for OpenGL context
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        
        window = glfwCreateWindow(width, height, "MuJoCo RGBD Capture", NULL, NULL);
        if (!window) {
            glfwTerminate();
            throw std::runtime_error("Failed to create GLFW window");
        }
        
        glfwMakeContextCurrent(window);
        std::cout << "OpenGL context created successfully!" << std::endl;
        
        // Load model from XML file
        char error[1000] = "Could not load model";
        std::cout << "Loading model from: " << model_path << std::endl;
        
        model = mj_loadXML(model_path, NULL, error, 1000);
        if (!model) {
            throw std::runtime_error("Failed to load model: " + std::string(error));
        }
        
        std::cout << "Model loaded successfully!" << std::endl;
        
        // Create data
        data = mj_makeData(model);
        std::cout << "Data created successfully!" << std::endl;
        
        // Initialize visualization structures
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);
        
        // Set up passive viewer camera
        cam.type = mjCAMERA_FREE;
        cam.fixedcamid = -1;
        cam.trackbodyid = -1;
        
        // Make scene
        mjv_makeScene(model, &scn, 2000);
        std::cout << "Scene created successfully!" << std::endl;
        
        // Initialize OpenGL context
        mjr_makeContext(model, &con, mjFONTSCALE_150);
        std::cout << "MuJoCo OpenGL context created successfully!" << std::endl;
        
        // Find camera
        camera_id = mj_name2id(model, mjOBJ_CAMERA, "camera");
        if (camera_id == -1) {
            throw std::runtime_error("Camera 'camera' not found in model");
        }
        
        // Initialize buffers
        rgb_buffer.resize(width * height * 3);
        depth_buffer.resize(width * height);
        
        // Setup camera matrices
        setupCameraMatrices();
    }
    
    ~MuJoCoRGBDCapture() {
        mjr_freeContext(&con);
        mjv_freeScene(&scn);
        mj_deleteData(data);
        mj_deleteModel(model);
        
        if (window) {
            glfwDestroyWindow(window);
        }
        glfwTerminate();
    }
    
private:
    void setupCameraMatrices() {
        // Forward simulation to get camera data
        mj_forward(model, data);
        
        // Intrinsic matrix calculation
        double fov = model->cam_fovy[camera_id];
        double theta = fov * M_PI / 180.0;  // Convert to radians
        double fx = width / 2.0 / tan(theta / 2.0);
        double fy = height / 2.0 / tan(theta / 2.0);
        double cx = (width - 1) / 2.0;
        double cy = (height - 1) / 2.0;
        
        intrinsic_matrix(0, 0) = fx;
        intrinsic_matrix(0, 2) = cx;
        intrinsic_matrix(1, 1) = fy;
        intrinsic_matrix(1, 2) = cy;
        intrinsic_matrix(2, 2) = 1.0;
        
        // Extrinsic matrix calculation
        // Camera position
        Vec3 cam_pos(data->cam_xpos[camera_id * 3],
                    data->cam_xpos[camera_id * 3 + 1],
                    data->cam_xpos[camera_id * 3 + 2]);
        
        // Camera rotation matrix (3x3)
        Mat3 cam_rot;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cam_rot(i, j) = data->cam_xmat[camera_id * 9 + i * 3 + j];
            }
        }
        
        // Build extrinsic matrix [R^T | -R^T * t]
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                extrinsic_matrix(i, j) = cam_rot(j, i);  // Transpose
            }
        }
        
        // Translation part: -R^T * t
        extrinsic_matrix(0, 3) = -(cam_rot(0, 0) * cam_pos.x + cam_rot(1, 0) * cam_pos.y + cam_rot(2, 0) * cam_pos.z);
        extrinsic_matrix(1, 3) = -(cam_rot(0, 1) * cam_pos.x + cam_rot(1, 1) * cam_pos.y + cam_rot(2, 1) * cam_pos.z);
        extrinsic_matrix(2, 3) = -(cam_rot(0, 2) * cam_pos.x + cam_rot(1, 2) * cam_pos.y + cam_rot(2, 2) * cam_pos.z);
    }
    
    void renderRGBD() {
        // Update scene
        mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        
        // Set viewport
        mjrRect viewport = {0, 0, width, height};
        
        // Render RGB
        mjr_render(viewport, &scn, &con);
        mjr_readPixels(rgb_buffer.data(), NULL, viewport, &con);
        
        // Render depth
        mjr_render(viewport, &scn, &con);
        mjr_readPixels(NULL, depth_buffer.data(), viewport, &con);
    }
    
    void renderViewer() {
        // Update scene for viewer
        mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        
        // Set viewport
        mjrRect viewport = {0, 0, width, height};
        
        // Render for viewer
        mjr_render(viewport, &scn, &con);
    }
    
public:
    std::vector<Vec6> rgbdToPointCloud(double depth_trunc = 20.0) {
        std::vector<Vec6> point_cloud;
        
        for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
                int idx = row * width + col;
                double depth = depth_buffer[idx];
                
                // Check if depth is valid
                if (depth <= 0 || depth >= depth_trunc) continue;
                
                // Unproject to camera coordinates
                double x = depth * (col - intrinsic_matrix(0, 2)) / intrinsic_matrix(0, 0);
                double y = depth * (row - intrinsic_matrix(1, 2)) / intrinsic_matrix(1, 1);
                double z = depth;
                
                // Transform to world coordinates using extrinsic matrix
                double world_x = extrinsic_matrix(0, 0) * x + extrinsic_matrix(0, 1) * y + 
                                extrinsic_matrix(0, 2) * z + extrinsic_matrix(0, 3);
                double world_y = extrinsic_matrix(1, 0) * x + extrinsic_matrix(1, 1) * y + 
                                extrinsic_matrix(1, 2) * z + extrinsic_matrix(1, 3);
                double world_z = extrinsic_matrix(2, 0) * x + extrinsic_matrix(2, 1) * y + 
                                extrinsic_matrix(2, 2) * z + extrinsic_matrix(2, 3);
                
                // Get color (RGB values are usually stored as BGR in OpenGL)
                int rgb_idx = idx * 3;
                double r = rgb_buffer[rgb_idx + 2] / 255.0;     // Red
                double g = rgb_buffer[rgb_idx + 1] / 255.0;     // Green
                double b = rgb_buffer[rgb_idx] / 255.0;         // Blue
                
                point_cloud.emplace_back(world_x, world_y, world_z, r, g, b);
            }
        }
        
        return point_cloud;
    }
    
    void simulate(double sim_time = 10.0) {
        std::vector<std::vector<Vec6>> point_clouds;
        
        // Reset data
        mj_resetData(model, data);
        
        while (data->time < sim_time && !glfwWindowShouldClose(window)) {
            // Step simulation
            mj_step(model, data);
            
            // Capture frame at specified fps
            if (point_clouds.size() < static_cast<size_t>(data->time * fps)) {
                renderRGBD();
                std::vector<Vec6> pc = rgbdToPointCloud();
                point_clouds.push_back(pc);
                
                std::cout << "Frame " << point_clouds.size() << " captured at time " 
                         << data->time << " with " << pc.size() << " points" << std::endl;
            }
            
            // Render viewer
            renderViewer();
            
            // Update window
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
        
        // Save point clouds to files (simple format)
        for (size_t i = 0; i < point_clouds.size(); i++) {
            savePointCloud(point_clouds[i], "pointcloud_" + std::to_string(i) + ".xyz");
        }
    }
    
private:
    void savePointCloud(const std::vector<Vec6>& pc, const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }
        
        for (const auto& point : pc) {
            file << point.x << " " << point.y << " " << point.z << " " 
                 << point.r << " " << point.g << " " << point.b << std::endl;
        }
        
        file.close();
        std::cout << "Saved " << pc.size() << " points to " << filename << std::endl;
    }
};

int main(int argc, char* argv[]) {
    try {
        const char* model_path = (argc > 1) ? argv[1] : "../model.xml";
        
        MuJoCoRGBDCapture capture(model_path, 640, 480, 30.0);
        capture.simulate(10.0);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}