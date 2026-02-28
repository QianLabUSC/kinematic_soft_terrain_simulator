// Chrono Packages
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_parsers/ChParserURDF.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "sim_tools/VariableSoilParams.h"
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "foxglove_msgs/msg/grid.hpp"
#include "trusses_custom_interfaces/msg/spatial_measurement.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// Standard Packages
#include <cstdlib>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <csignal>
#include <atomic>
#include <filesystem>
#include <mutex>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
// Sim Tools
#include "sim_tools/RHexFactory.h"
#include "sim_tools/RHex.h"

using namespace chrono;
using namespace chrono::parsers;
using namespace chrono::irrlicht;

std::atomic_bool g_terminate_requested(false);

void signal_handler(int signum) {
    if (signum == SIGINT) {
        g_terminate_requested.store(true);
        // RCLCPP_INFO(rclcpp::get_logger("main"), "Termination signal (CTRL-c) received. Shutting down...");
    }
}

// Bridge node: subscribes to spirit/planner_twist_command, publishes spirit/current_pose and spirit/spatial_measurements
class SpiritTwistBridge : public rclcpp::Node {
public:
    SpiritTwistBridge() : Node("spirit_chrono_bridge") {
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "spirit/planner_twist_command", 10,
            std::bind(&SpiritTwistBridge::twist_callback, this, std::placeholders::_1));
        ground_truth_map_sub_ = this->create_subscription<foxglove_msgs::msg::Grid>(
            "/ground_truth_map", 10,
            std::bind(&SpiritTwistBridge::ground_truth_map_callback, this, std::placeholders::_1));
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("spirit/current_pose", 10);
        spatial_measurements_pub_ = this->create_publisher<trusses_custom_interfaces::msg::SpatialMeasurement>("spirit/spatial_measurements", 10);
        robot_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "spirit/current_pose_marker", 10);
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        bool first_command = false;
        double linear_x = 0.0;
        double linear_y = 0.0;
        double angular_z = 0.0;
        {
            std::lock_guard<std::mutex> lock(twist_mutex_);
            first_command = !has_twist_;
            last_twist_ = *msg;
            has_twist_ = true;
            linear_x = last_twist_.linear.x;
            linear_y = last_twist_.linear.y;
            angular_z = last_twist_.angular.z;
        }

        if (first_command) {
            RCLCPP_INFO(this->get_logger(),
                        "Received first /spirit/planner_twist_command message.");
        }
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Receiving /spirit/planner_twist_command: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f",
            linear_x, linear_y, angular_z);
    }

    bool get_twist(double& linear_x, double& angular_z) {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        if (!has_twist_) return false;
        linear_x = last_twist_.linear.x;
        angular_z = last_twist_.angular.z;
        return true;
    }

    bool export_ground_truth_texture_png(const std::string& texture_path,
                                         uint64_t& generation_out,
                                         uint8_t& min_channel_out,
                                         uint8_t& max_channel_out) {
        foxglove_msgs::msg::Grid map_msg;
        {
            std::lock_guard<std::mutex> lock(ground_truth_map_mutex_);
            if (!has_ground_truth_map_) {
                return false;
            }
            map_msg = latest_ground_truth_map_;
            generation_out = ground_truth_map_generation_;
        }

        const int width = static_cast<int>(map_msg.column_count);
        const int row_stride = static_cast<int>(map_msg.row_stride);
        const int cell_stride = static_cast<int>(map_msg.cell_stride);
        if (width <= 0 || row_stride <= 0 || cell_stride <= 0 || map_msg.data.empty()) {
            return false;
        }

        const int height = static_cast<int>(map_msg.data.size() / row_stride);
        if (height <= 0) {
            return false;
        }

        int red_offset = 1;
        int green_offset = 2;
        int blue_offset = 3;
        for (const auto& field : map_msg.fields) {
            if (field.name == "red") {
                red_offset = static_cast<int>(field.offset);
            } else if (field.name == "green") {
                green_offset = static_cast<int>(field.offset);
            } else if (field.name == "blue") {
                blue_offset = static_cast<int>(field.offset);
            }
        }

        const int max_offset = std::max({red_offset, green_offset, blue_offset});
        if (max_offset >= cell_stride) {
            return false;
        }

        cv::Mat texture(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

        uint8_t min_channel = std::numeric_limits<uint8_t>::max();
        uint8_t max_channel = std::numeric_limits<uint8_t>::lowest();
        bool has_valid_pixels = false;

        for (int row = 0; row < height; ++row) {
            // Grid rows are from bottom to top; images are top to bottom.
            const int image_row = height - 1 - row;
            auto* pixel_row = texture.ptr<cv::Vec3b>(image_row);
            for (int col = 0; col < width; ++col) {
                const size_t base = static_cast<size_t>(row) * static_cast<size_t>(row_stride) +
                                    static_cast<size_t>(col) * static_cast<size_t>(cell_stride);
                if (base + static_cast<size_t>(max_offset) >= map_msg.data.size()) {
                    continue;
                }

                const uint8_t red = map_msg.data[base + static_cast<size_t>(red_offset)];
                const uint8_t green = map_msg.data[base + static_cast<size_t>(green_offset)];
                const uint8_t blue = map_msg.data[base + static_cast<size_t>(blue_offset)];
                min_channel = std::min(min_channel, std::min(red, std::min(green, blue)));
                max_channel = std::max(max_channel, std::max(red, std::max(green, blue)));
                has_valid_pixels = true;

                // Custom darker colormap:
                // previous "red" intensities are mapped to yellow, and
                // previous "blue" intensities are mapped to green.
                constexpr double darkness = 0.55;
                const double red_d = static_cast<double>(red);
                const double blue_d = static_cast<double>(blue);
                const uint8_t mapped_r = static_cast<uint8_t>(
                    std::clamp(darkness * red_d, 0.0, 255.0));
                const uint8_t mapped_g = static_cast<uint8_t>(
                    std::clamp(darkness * std::max(red_d, blue_d), 0.0, 255.0));
                const uint8_t mapped_b = 0;

                // OpenCV uses BGR channel order.
                pixel_row[col] = cv::Vec3b(mapped_b, mapped_g, mapped_r);
            }
        }

        if (!has_valid_pixels) {
            return false;
        }

        if (!cv::imwrite(
                texture_path, texture,
                {cv::IMWRITE_PNG_COMPRESSION, 3})) {
            return false;
        }

        min_channel_out = min_channel;
        max_channel_out = max_channel;
        return true;
    }

    void publish_pose(const ChVector3d& position, const ChQuaternion<>& quat) {
        geometry_msgs::msg::Pose msg;
        msg.position.x = position.x();
        msg.position.y = position.y();
        msg.position.z = position.z();
        msg.orientation.w = quat.e0();
        msg.orientation.x = quat.e1();
        msg.orientation.y = quat.e2();
        msg.orientation.z = quat.e3();
        pose_pub_->publish(msg);
    }

    void publish_spatial_measurement(int32_t leg_idx, const ChVector3d& position, float stiffness_value) {
        trusses_custom_interfaces::msg::SpatialMeasurement msg;
        msg.leg_idx = leg_idx;
        msg.position.x = static_cast<float>(position.x());
        msg.position.y = static_cast<float>(position.y());
        msg.position.z = static_cast<float>(position.z());
        msg.value = stiffness_value;
        msg.uncertainty = 0.0f;
        msg.unit = "unitless";
        msg.source_name = "chrono_sim";
        rclcpp::Time t = this->now();
        int64_t nsec = t.nanoseconds();
        msg.time.sec = static_cast<int32_t>(nsec / 1000000000);
        msg.time.nanosec = static_cast<uint32_t>(nsec % 1000000000);
        spatial_measurements_pub_->publish(msg);
    }

    void publish_robot_position_marker(const ChVector3d& position,
                                       const ChQuaternion<>& quat) {
        visualization_msgs::msg::MarkerArray marker_array;
        const auto stamp = this->now();

        visualization_msgs::msg::Marker body_marker;
        body_marker.header.frame_id = "map";
        body_marker.header.stamp = stamp;
        body_marker.ns = "robot";
        body_marker.id = 0;
        body_marker.type = visualization_msgs::msg::Marker::CUBE;
        body_marker.action = visualization_msgs::msg::Marker::ADD;
        body_marker.pose.position.x = position.x();
        body_marker.pose.position.y = position.y();
        body_marker.pose.position.z = position.z();
        body_marker.pose.orientation.w = quat.e0();
        body_marker.pose.orientation.x = quat.e1();
        body_marker.pose.orientation.y = quat.e2();
        body_marker.pose.orientation.z = quat.e3();
        body_marker.scale.x = 0.4;
        body_marker.scale.y = 0.3;
        body_marker.scale.z = 0.2;
        body_marker.color.r = 0.0f;
        body_marker.color.g = 1.0f;
        body_marker.color.b = 1.0f;
        body_marker.color.a = 1.0f;
        marker_array.markers.push_back(body_marker);

        const double qw = quat.e0();
        const double qx = quat.e1();
        const double qy = quat.e2();
        const double qz = quat.e3();
        const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                      1.0 - 2.0 * (qy * qy + qz * qz));

        visualization_msgs::msg::Marker head_marker;
        head_marker.header.frame_id = "map";
        head_marker.header.stamp = stamp;
        head_marker.ns = "robot";
        head_marker.id = 1;
        head_marker.type = visualization_msgs::msg::Marker::CUBE;
        head_marker.action = visualization_msgs::msg::Marker::ADD;
        const double offset_distance = 0.14;
        head_marker.pose.position.x = position.x() + offset_distance * std::cos(yaw);
        head_marker.pose.position.y = position.y() + offset_distance * std::sin(yaw);
        head_marker.pose.position.z = position.z() + 0.03;
        head_marker.pose.orientation.w = quat.e0();
        head_marker.pose.orientation.x = quat.e1();
        head_marker.pose.orientation.y = quat.e2();
        head_marker.pose.orientation.z = quat.e3();
        head_marker.scale.x = 0.1;
        head_marker.scale.y = 0.25;
        head_marker.scale.z = 0.21;
        head_marker.color.r = 1.0f;
        head_marker.color.g = 1.0f;
        head_marker.color.b = 0.0f;
        head_marker.color.a = 1.0f;
        marker_array.markers.push_back(head_marker);

        robot_marker_pub_->publish(marker_array);
    }

private:
    void ground_truth_map_callback(const foxglove_msgs::msg::Grid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(ground_truth_map_mutex_);
        latest_ground_truth_map_ = *msg;
        has_ground_truth_map_ = true;
        ++ground_truth_map_generation_;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<foxglove_msgs::msg::Grid>::SharedPtr ground_truth_map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<trusses_custom_interfaces::msg::SpatialMeasurement>::SharedPtr spatial_measurements_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_marker_pub_;
    foxglove_msgs::msg::Grid latest_ground_truth_map_;
    bool has_ground_truth_map_{false};
    uint64_t ground_truth_map_generation_{0};
    std::mutex ground_truth_map_mutex_;
    geometry_msgs::msg::Twist last_twist_;
    bool has_twist_ = false;
    std::mutex twist_mutex_;
};

struct GroundTruthBounds {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
};

struct DriveSimInitialPose {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    bool has_x{false};
    bool has_y{false};
    bool has_theta{false};
};

std::string trim_copy(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

bool parse_double_value(const std::string& value, double& out) {
    try {
        size_t parsed = 0;
        out = std::stod(value, &parsed);
        return parsed > 0;
    } catch (const std::exception&) {
        return false;
    }
}

bool load_drive_sim_initial_pose_from_yaml(const rclcpp::Node::SharedPtr& node,
                                           DriveSimInitialPose& pose) {
    const char* home = std::getenv("HOME");
    if (!home) {
        RCLCPP_WARN(node->get_logger(),
                    "HOME env var not set. Using fallback initial pose (0, 0, 0).");
        return false;
    }

    const std::filesystem::path config_path =
        std::filesystem::path(home) /
        "ros2_ws/src/Scout_Interface/src/spirit_high_launch/config/safe_scouting.yaml";
    std::ifstream file(config_path);
    if (!file.is_open()) {
        RCLCPP_WARN(node->get_logger(),
                    "Cannot open '%s'. Using fallback initial pose (0, 0, 0).",
                    config_path.string().c_str());
        return false;
    }

    bool in_drive_sim = false;
    bool in_ros_params = false;
    size_t drive_sim_indent = 0;
    size_t ros_params_indent = 0;

    std::string raw_line;
    while (std::getline(file, raw_line)) {
        const auto comment_pos = raw_line.find('#');
        const std::string no_comment =
            (comment_pos == std::string::npos) ? raw_line : raw_line.substr(0, comment_pos);
        const auto content_pos = no_comment.find_first_not_of(" \t");
        if (content_pos == std::string::npos) {
            continue;
        }

        const size_t indent = content_pos;
        const std::string trimmed = trim_copy(no_comment);
        const auto colon_pos = trimmed.find(':');
        if (colon_pos == std::string::npos) {
            continue;
        }

        const std::string key = trim_copy(trimmed.substr(0, colon_pos));
        const std::string value = trim_copy(trimmed.substr(colon_pos + 1));
        const bool is_section = value.empty();

        if (!in_drive_sim) {
            if (indent == 0 && key == "drive_sim" && is_section) {
                in_drive_sim = true;
                drive_sim_indent = indent;
            }
            continue;
        }

        if (indent <= drive_sim_indent && key != "drive_sim" && is_section) {
            in_drive_sim = false;
            in_ros_params = false;
            continue;
        }

        if (!in_ros_params) {
            if (key == "ros__parameters" && is_section) {
                in_ros_params = true;
                ros_params_indent = indent;
            }
            continue;
        }

        if (indent <= ros_params_indent) {
            in_ros_params = false;
            continue;
        }

        double parsed_value = 0.0;
        if (!parse_double_value(value, parsed_value)) {
            continue;
        }

        if (key == "initial_x") {
            pose.x = parsed_value;
            pose.has_x = true;
        } else if (key == "initial_y") {
            pose.y = parsed_value;
            pose.has_y = true;
        } else if (key == "initial_theta") {
            pose.theta = parsed_value;
            pose.has_theta = true;
        }
    }

    if (!pose.has_x || !pose.has_y) {
        RCLCPP_WARN(node->get_logger(),
                    "initial_x/initial_y not found in '%s'. Using fallback initial pose (0, 0, 0).",
                    config_path.string().c_str());
        return false;
    }

    if (!pose.has_theta) {
        pose.theta = 0.0;
    }

    RCLCPP_INFO(node->get_logger(),
                "Using drive_sim initial pose from YAML: x=%.3f, y=%.3f, theta=%.3f",
                pose.x, pose.y, pose.theta);
    return true;
}

bool get_ground_truth_bounds(const rclcpp::Node::SharedPtr& node,
                             GroundTruthBounds& bounds) {
    auto param_client =
        std::make_shared<rclcpp::SyncParametersClient>(node, "/ground_truth_server");

    if (!param_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(node->get_logger(),
                    "ground_truth_server parameter service unavailable. "
                    "Using fallback terrain bounds [0,10]x[0,10].");
        return false;
    }

    const std::vector<std::string> names{
        "area_min_x", "area_max_x", "area_min_y", "area_max_y"};
    const auto params = param_client->get_parameters(names);
    if (params.size() != names.size()) {
        RCLCPP_WARN(node->get_logger(),
                    "Failed to read all ground_truth_server bounds. "
                    "Using fallback terrain bounds [0,10]x[0,10].");
        return false;
    }

    try {
        bounds.min_x = params[0].as_double();
        bounds.max_x = params[1].as_double();
        bounds.min_y = params[2].as_double();
        bounds.max_y = params[3].as_double();
    } catch (const std::exception& e) {
        RCLCPP_WARN(node->get_logger(),
                    "Invalid ground_truth_server bound type (%s). "
                    "Using fallback terrain bounds [0,10]x[0,10].",
                    e.what());
        return false;
    }

    if (bounds.max_x <= bounds.min_x || bounds.max_y <= bounds.min_y) {
        RCLCPP_WARN(node->get_logger(),
                    "Invalid ground_truth_server bounds x:[%.3f, %.3f], y:[%.3f, %.3f]. "
                    "Using fallback terrain bounds [0,10]x[0,10].",
                    bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y);
        return false;
    }

    RCLCPP_INFO(node->get_logger(),
                "Using ground_truth_server bounds x:[%.3f, %.3f], y:[%.3f, %.3f]",
                bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y);
    return true;
}

double test(int argc, char* argv[], double bodymass, double legmass,
            std::string stiffness, double stiffness_scale, double counterweight_mass) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    SetChronoDataPath("/usr/local/share/chrono/data/");

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
    sys.SetNumThreads(4, 8, 1);
    collsys->SetNumThreads(4);
    sys.SetCollisionSystem(collsys);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    double step_size = 0.004;
    const int sim_substeps_per_frame = 4;  // >1 speeds up simulated time per rendered frame
    const bool enable_shadows = false;     // shadows are expensive; disable for faster sim

    GroundTruthBounds bounds{0.0, 10.0, 0.0, 10.0};
    DriveSimInitialPose initial_pose{};
    {
        auto config_node = std::make_shared<rclcpp::Node>("spirit_chrono_terrain_config");
        get_ground_truth_bounds(config_node, bounds);
        load_drive_sim_initial_pose_from_yaml(config_node, initial_pose);
    }

    const double terrain_length = std::max(bounds.max_x - bounds.min_x, 1e-3);
    const double terrain_width = std::max(bounds.max_y - bounds.min_y, 1e-3);
    const ChVector3d terrain_center(
        0.5 * (bounds.min_x + bounds.max_x),
        0.5 * (bounds.min_y + bounds.max_y),
        0.0);

    vehicle::SCMTerrain mterrain(&sys);
    double mesh_resolution = 0.03;
    mterrain.SetPlane(ChCoordsys<>(terrain_center, QUNIT));
    mterrain.Initialize(terrain_length, terrain_width, mesh_resolution);

    mterrain.EnableBulldozing(true);
    mterrain.SetBulldozingParameters(55, 1, 5, 6);

    // Visualization
    // Keep filled mesh so texture from /ground_truth_map is visible.
    mterrain.SetMeshWireframe(true);
    // Disable sinkage colormap to avoid overriding texture appearance.
    mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.05);
    // Bridge: subscribe to spirit/planner_twist_command and /ground_truth_map.
    auto bridge = std::make_shared<SpiritTwistBridge>();

    // Save texture PNG from /ground_truth_map and set terrain texture once before sim loop.
    const char* home_env = std::getenv("HOME");
    const std::string ground_truth_texture_path =
        (home_env != nullptr)
            ? (std::string(home_env) + "/ros2_ws/src/trusses_chrono_sim/meshes/output.png")
            : std::string("/tmp/ground_truth_map_texture.png");
    {
        std::error_code ec;
        const auto parent = std::filesystem::path(ground_truth_texture_path).parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent, ec);
        }
    }

    RCLCPP_INFO(bridge->get_logger(),
                "Waiting for first /ground_truth_map message to build terrain PNG texture at: %s",
                ground_truth_texture_path.c_str());
    bool texture_ready = false;
    while (rclcpp::ok()) {
        rclcpp::spin_some(bridge);
        uint64_t map_generation = 0;
        uint8_t min_channel = 0;
        uint8_t max_channel = 0;
        if (bridge->export_ground_truth_texture_png(ground_truth_texture_path, map_generation,
                                                    min_channel, max_channel)) {
            mterrain.SetTexture(ground_truth_texture_path, 1.0, -1.0);
            mterrain.SetColor(ChColor(0.5, 0.5, 0.5));
            RCLCPP_INFO(
                bridge->get_logger(),
                "Applied initial /ground_truth_map texture (generation=%lu, channel_range=[%u,%u]).",
                static_cast<unsigned long>(map_generation),
                static_cast<unsigned int>(min_channel),
                static_cast<unsigned int>(max_channel));
            texture_ready = true;
            break;
        }
        RCLCPP_WARN_THROTTLE(
            bridge->get_logger(), *bridge->get_clock(), 3000,
            "Still waiting for /ground_truth_map before starting simulation.");
        if (g_terminate_requested.load()) {
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    if (!rclcpp::ok() || g_terminate_requested.load() || !texture_ready) {
        RCLCPP_WARN(bridge->get_logger(),
                    "Simulation stopped before initial /ground_truth_map texture became available.");
        rclcpp::shutdown();
        return 0.0;
    }

    // Variable soil params
    std::string stiffness_map_filepath =
        std::string(getenv("HOME")) + "/ros2_ws/src/trusses_chrono_sim/scripts/output_stiffness.csv";
    auto my_params = chrono_types::make_shared<VariableSoilParams>(stiffness_map_filepath, stiffness_scale);
    // Use robot body center for service-based stiffness queries.
    my_params->SetServiceQueryRobotCenter(initial_pose.x, initial_pose.y);
    mterrain.RegisterSoilParametersCallback(my_params);

    // ----- Robot initial position and orientation (set here) -----
    // Position (x, y, z) in meters; z is height above terrain.
    ChVector3d RHex_position(initial_pose.x, initial_pose.y, 0.09);
    // Initial yaw in radians (0 = facing +X; use atan2(dy,dx) or M_PI/2 for +Y, etc.).
    double start_yaw = initial_pose.theta;
    // -------------------------------------------------------------

    RHexFactory RHexFactory(&sys);
    auto Rhex = RHexFactory.BuildRHex(RHex_position, QuatFromAngleZ(start_yaw),
                                      {}, bodymass, legmass, counterweight_mass);
    Rhex->AddMovingPatchesAndCollisions(mterrain);
    
    // Set RHex PID gains (Kp, Ki, Kd)
    // Reduced from aggressive values to mitigate oscillation/shaking.
    Rhex->SetLegGains(800.0, 0.0, 10.0);

    // Visualization system
    auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(1600, 1200);
    vis_irr->SetWindowTitle("RHex Spirit Twist Control");
    vis_irr->SetCameraVertical(CameraVerticalDir::Z);
    vis_irr->Initialize();
    vis_irr->AddSkyBox();
    vis_irr->AddTypicalLights();
    const double camera_extent = std::max(terrain_length, terrain_width);
    vis_irr->AddCamera(
        ChVector3d(terrain_center.x(), terrain_center.y() - 1.2 * camera_extent, 0.6 * camera_extent),
        ChVector3d(terrain_center.x(), terrain_center.y(), 0.8));
    if (enable_shadows) {
        vis_irr->EnableShadows();
    }
    vis_irr->AttachSystem(&sys);

    if (mterrain.GetMesh()) {
        vis_irr->AddVisualModel(mterrain.GetMesh(), ChFrame<>());
    }
    std::shared_ptr<ChVisualSystem> vis = vis_irr;

    double simulation_time = sys.GetChTime();

    // Twist -> leg velocity mapping (gains; tune as needed)
    const double linear_gain = 5.0 * M_PI;   // linear.x (m/s) -> leg rad/s scale
    const double angular_gain = M_PI;         // angular.z (rad/s) -> turn leg rad/s
    const double leg_movement_start_time = 0.1;
    const auto wall_start_time = std::chrono::steady_clock::now();

    while (rclcpp::ok() && vis->Run()) {
        // Process twist commands from spirit/planner_twist_command
        rclcpp::spin_some(bridge);
        Rhex->HandleROS();

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        ChVector3d current_pos = Rhex->m_body->GetPos();
        ChQuaternion<> current_rot = Rhex->m_body->GetRot();
        my_params->SetServiceQueryRobotCenter(current_pos.x(), current_pos.y());

        // Drive legs from twist (spirit/planner_twist_command)
        double linear_x = 0.0, angular_z = 0.0;
        bool has_cmd = bridge->get_twist(linear_x, angular_z);
        double cmd_left = 0.0;
        double cmd_right = 0.0;

        if (has_cmd) {
            // Map twist to leg angular velocities: forward from linear.x, turn from angular.z
            const double forward_leg = -linear_x * linear_gain;  // body +X forward => negative leg velocity
            const double turn_leg = angular_z * angular_gain;
            cmd_left = forward_leg + turn_leg;
            cmd_right = forward_leg - turn_leg;
        }

        for (int substep = 0; substep < sim_substeps_per_frame; ++substep) {
            // Apply control every physics step for smoother behavior.
            if (simulation_time < leg_movement_start_time) {
                for (int i = 0; i < 6; i++) {
                    Rhex->SetLegDesiredPosition(i, Rhex->m_leg_motors[i]->GetMotorAngle());
                }
            } else if (has_cmd) {
                Rhex->SetLegDesiredVelocity(0, cmd_left);
                Rhex->SetLegDesiredVelocity(2, cmd_left);
                Rhex->SetLegDesiredVelocity(4, cmd_left);
                Rhex->SetLegDesiredVelocity(1, cmd_right);
                Rhex->SetLegDesiredVelocity(3, cmd_right);
                Rhex->SetLegDesiredVelocity(5, cmd_right);
            } else {
                for (int i = 0; i < 6; i++) {
                    Rhex->SetLegDesiredPosition(i, Rhex->m_leg_motors[i]->GetMotorAngle());
                }
            }

            vis->BindAll();
            sys.DoStepDynamics(step_size);
            simulation_time = sys.GetChTime();
        }

        current_pos = Rhex->m_body->GetPos();
        current_rot = Rhex->m_body->GetRot();

        // Publish current pose to spirit/current_pose
        bridge->publish_pose(current_pos, current_rot);
        bridge->publish_robot_position_marker(current_pos, current_rot);

        // Leg 0 (front left): stiffness at leg position as SpatialMeasurement on spirit/spatial_measurements
        const int leg0 = 0;
        ChVector3d leg0_pos = Rhex->m_legs[leg0]->GetPos();
        const double bekker_kphi = my_params->ComputeBekkerKphiAt(leg0_pos);
        const double normalized_publish =
            VariableSoilParams::BekkerKphiToServiceNormalized(bekker_kphi);
        const float stiffness_at_leg0 =
            static_cast<float>(std::clamp(normalized_publish, 0.0, 1.0));
        bridge->publish_spatial_measurement(leg0, leg0_pos, stiffness_at_leg0);

        const double left_actual =
            (Rhex->m_leg_motors[0]->GetMotorAngleDt() +
             Rhex->m_leg_motors[2]->GetMotorAngleDt() +
             Rhex->m_leg_motors[4]->GetMotorAngleDt()) / 3.0;
        const double right_actual =
            (Rhex->m_leg_motors[1]->GetMotorAngleDt() +
             Rhex->m_leg_motors[3]->GetMotorAngleDt() +
             Rhex->m_leg_motors[5]->GetMotorAngleDt()) / 3.0;
        RCLCPP_INFO_THROTTLE(
            bridge->get_logger(), *bridge->get_clock(), 1000,
            "Twist->Leg: has_cmd=%d, cmd(vx=%.3f,wz=%.3f), target(left=%.3f,right=%.3f), actual(left=%.3f,right=%.3f) rad/s",
            has_cmd ? 1 : 0, linear_x, angular_z, cmd_left, cmd_right,
            left_actual, right_actual);

        const double wall_elapsed_s = std::max(
            1e-9, std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start_time).count());
        const double rtf = simulation_time / wall_elapsed_s;
        RCLCPP_INFO_THROTTLE(
            bridge->get_logger(), *bridge->get_clock(), 1000,
            "SimTime: %.2f s | Wall: %.2f s | RTF: %.2f | step=%.4f x %d/frame",
            simulation_time, wall_elapsed_s, rtf, step_size, sim_substeps_per_frame);

        if (g_terminate_requested.load()) break;
    }

    rclcpp::shutdown();
    return 0.0;
}

int main(int argc, char* argv[]) {
    double bodymass = 80.0;
    double counterweight_mass = 0.0;
    double legmass = 0.39;
    double stiffness_scale = 1.0;
    std::string stiffness = "50";
    double velocity =
        test(argc, argv, bodymass, legmass, stiffness, stiffness_scale, counterweight_mass);
    return 0;
}
