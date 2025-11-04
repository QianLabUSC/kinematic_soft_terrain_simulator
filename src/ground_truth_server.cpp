#include <memory>
#include <vector>
#include <random>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <chrono>
#include <filesystem>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "safe_scout_simulator/srv/sample_ground_truth.hpp"
#include "foxglove_msgs/msg/grid.hpp"
#include "foxglove_msgs/msg/packed_element_field.hpp"
#include "foxglove_msgs/msg/vector2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "builtin_interfaces/msg/time.hpp"

struct Gaussian2D {
    double mean_x;
    double mean_y;
    double std_x;        // Standard deviation in x direction (before rotation)
    double std_y;        // Standard deviation in y direction (before rotation)
    double rotation;     // Rotation angle in radians (counter-clockwise from x-axis)
    double amplitude;
};

class DenseGroundTruthGenerator {
public:
    DenseGroundTruthGenerator(std::vector<Gaussian2D> gaussians,
                              double area_min_x, double area_max_x,
                              double area_min_y, double area_max_y,
                              double lipschitz_constant,
                              int random_seed)
        : area_min_x_(area_min_x), area_max_x_(area_max_x),
          area_min_y_(area_min_y), area_max_y_(area_max_y),
          lipschitz_constant_(lipschitz_constant),
          random_seed_(random_seed),
          gaussians_(std::move(gaussians)) {}

    DenseGroundTruthGenerator(double area_min_x, double area_max_x,
                              double area_min_y, double area_max_y,
                              int num_gaussians, double lipschitz_constant,
                              int random_seed)
        : area_min_x_(area_min_x), area_max_x_(area_max_x),
          area_min_y_(area_min_y), area_max_y_(area_max_y),
          lipschitz_constant_(lipschitz_constant),
          random_seed_(random_seed) {

        generateGaussians(num_gaussians);
    }

    double sample(double x, double y) const {
        double value = 0.0;

        // Sum contributions from all Gaussians
        for (const auto& gaussian : gaussians_) {
            // Translate to Gaussian center
            double dx = x - gaussian.mean_x;
            double dy = y - gaussian.mean_y;
            
            // Rotate coordinates to align with Gaussian's principal axes
            double cos_theta = std::cos(gaussian.rotation);
            double sin_theta = std::sin(gaussian.rotation);
            double dx_rot = dx * cos_theta + dy * sin_theta;   // Rotated x
            double dy_rot = -dx * sin_theta + dy * cos_theta;   // Rotated y
            
            // Compute elliptical Gaussian: exp(-(x_rot²/(2σ_x²) + y_rot²/(2σ_y²)))
            double var_x = gaussian.std_x * gaussian.std_x;
            double var_y = gaussian.std_y * gaussian.std_y;
            double exponent = -(dx_rot * dx_rot / (2.0 * var_x) + dy_rot * dy_rot / (2.0 * var_y));
            
            value += gaussian.amplitude * std::exp(exponent);
        }

        return value;
    }

private:
    static std::vector<std::string> splitCSVLine(const std::string &line) {
        std::vector<std::string> result;
        std::stringstream ss(line);
        std::string item;
        while (std::getline(ss, item, ',')) {
            // trim spaces
            size_t start = item.find_first_not_of(" \t\r\n");
            size_t end = item.find_last_not_of(" \t\r\n");
            if (start == std::string::npos) {
                result.emplace_back("");
            } else {
                result.emplace_back(item.substr(start, end - start + 1));
            }
        }
        return result;
    }

    static std::string toLower(const std::string &s) {
        std::string out(s);
        for (char &c : out) c = static_cast<char>(::tolower(c));
        return out;
    }

    static bool parseDouble(const std::string &s, double &out) {
        try {
            size_t idx = 0;
            out = std::stod(s, &idx);
            return idx > 0;
        } catch (...) {
            return false;
        }
    }

public:
    static bool loadGaussiansFromCSV(const std::string &filepath, 
                                     double area_min_x, double area_max_x,
                                     double area_min_y, double area_max_y,
                                     std::vector<Gaussian2D> &out_gaussians, 
                                     std::string &error) {
        std::ifstream file(filepath);
        std::string header_line;
        std::getline(file, header_line);
        auto headers = splitCSVLine(header_line);
        std::unordered_map<std::string, size_t> name_to_idx;
        for (size_t i = 0; i < headers.size(); ++i) {
            name_to_idx[toLower(headers[i])] = i;
        }

        auto findIdx = [&](const std::vector<std::string> &candidates) -> int {
            for (const auto &n : candidates) {
                auto it = name_to_idx.find(toLower(n));
                if (it != name_to_idx.end()) return static_cast<int>(it->second);
            }
            return -1;
        };

        int idx_x = findIdx({"gaussian_x", "mean_x", "x"});
        int idx_y = findIdx({"gaussian_y", "mean_y", "y"});
        int idx_std_x = findIdx({"std_x", "sigma_x"});
        int idx_std_y = findIdx({"std_y", "sigma_y"});
        int idx_rotation = findIdx({"rotation", "theta", "angle"});
        int idx_amp = findIdx({"amplitude", "amp", "a"});

        // Count total lines first
        std::string line;
        size_t line_count = 0;
        while (std::getline(file, line)) {
            if (!line.empty()) {
                line_count++;
            }
        }
        
        // Reset file to beginning (after header)
        file.clear();
        file.seekg(0, std::ios::beg);
        std::getline(file, line);  // Skip header

        std::vector<Gaussian2D> gaussians;
        for (size_t i = 0; i < line_count; ++i) {
            std::getline(file, line);
            auto cols = splitCSVLine(line);
            
            double x_norm = std::stod(cols[static_cast<size_t>(idx_x)]);
            double y_norm = std::stod(cols[static_cast<size_t>(idx_y)]);
            double std_x_norm = std::stod(cols[static_cast<size_t>(idx_std_x)]);
            double std_y_norm = std::stod(cols[static_cast<size_t>(idx_std_y)]);
            double rotation_val = std::stod(cols[static_cast<size_t>(idx_rotation)]);
            double amp = std::stod(cols[static_cast<size_t>(idx_amp)]);
            
            double rotation = (std::abs(rotation_val) > 2.0 * M_PI) ? 
                              rotation_val * M_PI / 180.0 : rotation_val;

            double area_width = area_max_x - area_min_x;
            double area_height = area_max_y - area_min_y;
            double area_diagonal = std::sqrt(area_width * area_width + area_height * area_height);
            
            double x = area_min_x + x_norm * area_width;
            double y = area_min_y + y_norm * area_height;
            double std_x = std_x_norm * area_width;
            double std_y = std_y_norm * area_height;

            Gaussian2D g; 
            g.mean_x = x; 
            g.mean_y = y; 
            g.std_x = std_x;
            g.std_y = std_y;
            g.rotation = rotation;
            g.amplitude = amp;
            gaussians.push_back(g);
        }

        out_gaussians = std::move(gaussians);
        return true;
    }
    void generateGaussians(int num_gaussians) {
        std::mt19937 gen;
        if (random_seed_ >= 0) {
            gen.seed(static_cast<unsigned int>(random_seed_));
        } else {
            std::random_device rd;
            gen.seed(rd());
        }
        std::uniform_real_distribution<> dis_x(area_min_x_, area_max_x_);
        std::uniform_real_distribution<> dis_y(area_min_y_, area_max_y_);

        // Standard deviation is related to Lipschitz constant
        // Smaller std_dev means higher local variation (higher Lipschitz constant)
        double area_width = area_max_x_ - area_min_x_;
        double area_height = area_max_y_ - area_min_y_;
        double area_diagonal = std::sqrt(area_width * area_width + area_height * area_height);

        // Base standard deviation inversely proportional to Lipschitz constant
        double base_std_dev = area_diagonal / (lipschitz_constant_ * std::sqrt(num_gaussians));

        std::uniform_real_distribution<> dis_std(base_std_dev * 0.5, base_std_dev * 1.5);
        std::uniform_real_distribution<> dis_amp(0.5, 2.0);
        std::uniform_real_distribution<> dis_rotation(0.0, 2.0 * M_PI);  // Random rotation [0, 2π]

        gaussians_.reserve(num_gaussians);
        for (int i = 0; i < num_gaussians; ++i) {
            Gaussian2D g;
            g.mean_x = dis_x(gen);
            g.mean_y = dis_y(gen);
            double std_val = dis_std(gen);
            g.std_x = std_val;  // Use same for both (circular) or can vary
            g.std_y = std_val;
            g.rotation = dis_rotation(gen);  // Random rotation
            g.amplitude = dis_amp(gen);
            gaussians_.push_back(g);
        }
    }

    std::vector<Gaussian2D> gaussians_;
    double area_min_x_, area_max_x_;
    double area_min_y_, area_max_y_;
    double lipschitz_constant_;
    int random_seed_;
};

class GroundTruthServerNode : public rclcpp::Node {
public:
    GroundTruthServerNode() : Node("ground_truth_server") {
        // Declare parameters
        this->declare_parameter<double>("area_min_x", 0.0);
        this->declare_parameter<double>("area_max_x", 100.0);
        this->declare_parameter<double>("area_min_y", 0.0);
        this->declare_parameter<double>("area_max_y", 100.0);
        this->declare_parameter<int>("num_gaussians", 100);
        this->declare_parameter<double>("lipschitz_constant", 10.0);
        this->declare_parameter<int>("random_seed", -1);
        this->declare_parameter<std::string>("map_file", std::string(""));
        this->declare_parameter<bool>("if_randomize", true);
        this->declare_parameter<double>("grid_resolution", 0.05);  // Cell size in meters

        // Get parameters
        double area_min_x = this->get_parameter("area_min_x").as_double();
        double area_max_x = this->get_parameter("area_max_x").as_double();
        double area_min_y = this->get_parameter("area_min_y").as_double();
        double area_max_y = this->get_parameter("area_max_y").as_double();
        int num_gaussians = this->get_parameter("num_gaussians").as_int();
        double lipschitz_constant = this->get_parameter("lipschitz_constant").as_double();
        int random_seed = this->get_parameter("random_seed").as_int();

        // Initialize ground truth generator (random if if_randomize=true, else CSV)
        std::string map_file = this->get_parameter("map_file").as_string();
        bool if_randomize = this->get_parameter("if_randomize").as_bool();

        // Resolve relative paths to absolute paths
        if (!map_file.empty() && !if_randomize) {
            std::filesystem::path map_path(map_file);
            // If path is relative, resolve it relative to package share directory
            if (!map_path.is_absolute()) {
                try {
                    std::string package_share_dir = ament_index_cpp::get_package_share_directory("safe_scout_simulator");
                    std::filesystem::path full_path = std::filesystem::path(package_share_dir) / "maps" / map_path;
                    
                    // If no extension, assume .csv
                    if (full_path.extension().empty()) {
                        full_path = full_path.string() + ".csv";
                    }
                    
                    map_file = full_path.string();
                    RCLCPP_INFO(this->get_logger(), "Resolved relative map path to: %s", map_file.c_str());
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Failed to resolve package path: %s. Using original path: %s", 
                               e.what(), map_file.c_str());
                }
            } else {
                // Even for absolute paths, add .csv if no extension
                std::filesystem::path abs_path(map_file);
                if (abs_path.extension().empty()) {
                    map_file = map_file + ".csv";
                    RCLCPP_INFO(this->get_logger(), "Added .csv extension to path: %s", map_file.c_str());
                }
            }
        }

        if (if_randomize) {
            generator_ = std::make_unique<DenseGroundTruthGenerator>(
                area_min_x, area_max_x, area_min_y, area_max_y,
                num_gaussians, lipschitz_constant, random_seed);
            setReportedNumGaussians(static_cast<size_t>(num_gaussians));
            RCLCPP_INFO(this->get_logger(), "Randomized terrain with %d gaussians", num_gaussians);
        } else {
            std::vector<Gaussian2D> gs;
            std::string err;
            DenseGroundTruthGenerator::loadGaussiansFromCSV(map_file, 
                                                             area_min_x, area_max_x,
                                                             area_min_y, area_max_y,
                                                             gs, err);
            generator_ = std::make_unique<DenseGroundTruthGenerator>(
                std::move(gs), area_min_x, area_max_x, area_min_y, area_max_y,
                lipschitz_constant, random_seed);
            setReportedNumGaussians(gs.size());
            RCLCPP_INFO(this->get_logger(), "Loaded %zu gaussians from '%s'",
                        generatorSize(), map_file.c_str());
        }

        // Create service
        service_ = this->create_service<safe_scout_simulator::srv::SampleGroundTruth>(
            "sample_ground_truth",
            std::bind(&GroundTruthServerNode::handleSampleRequest, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Store area bounds for grid generation
        area_min_x_ = area_min_x;
        area_max_x_ = area_max_x;
        area_min_y_ = area_min_y;
        area_max_y_ = area_max_y;
        grid_resolution_ = this->get_parameter("grid_resolution").as_double();

        // Create grid publisher
        grid_publisher_ = this->create_publisher<foxglove_msgs::msg::Grid>("ground_truth_map", 10);

        // Generate grid at initialization based on resolution
        RCLCPP_INFO(this->get_logger(), "Generating ground truth grid with resolution %.3f m...", grid_resolution_);
        generateGridMap();
        RCLCPP_INFO(this->get_logger(), "Grid map generated: %d x %d cells (%zu total)", 
                    grid_width_, grid_height_, grid_values_.size());

        // Create timer to publish grid at 0.5 Hz (every 2 seconds)
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000),  // 2 seconds = 0.5 Hz
            std::bind(&GroundTruthServerNode::publishGrid, this));

        RCLCPP_INFO(this->get_logger(), "Ground Truth Server initialized");
        RCLCPP_INFO(this->get_logger(), "Area: [%.2f, %.2f] x [%.2f, %.2f]",
                    area_min_x, area_max_x, area_min_y, area_max_y);
        RCLCPP_INFO(this->get_logger(), "Number of Gaussians: %zu", generatorSize());
        RCLCPP_INFO(this->get_logger(), "Lipschitz constant: %.2f", lipschitz_constant);
        if (random_seed >= 0) {
            RCLCPP_INFO(this->get_logger(), "Random seed: %d (deterministic)", random_seed);
        } else {
            RCLCPP_INFO(this->get_logger(), "Random seed: random_device (non-deterministic)");
        }
    }

private:
    size_t generatorSize() const {
        // We can't access internal vector size directly; re-sample a few points isn't helpful.
        // Instead, rely on sampling behavior; but for logging, we can approximate by probing.
        // Simplify: maintain a shadow count by checking sampled values is not reliable; 
        // here we just return 0 when unknown. For better logging, store size when constructing.
        // To achieve that, store a field.
        return reported_num_gaussians_;
    }

    void setReportedNumGaussians(size_t n) { reported_num_gaussians_ = n; }

    void handleSampleRequest(
        const std::shared_ptr<safe_scout_simulator::srv::SampleGroundTruth::Request> request,
        std::shared_ptr<safe_scout_simulator::srv::SampleGroundTruth::Response> response) {

        response->value = generator_->sample(request->x, request->y);

        RCLCPP_DEBUG(this->get_logger(), "Sampled (%.2f, %.2f) = %.4f",
                     request->x, request->y, response->value);
    }

    void generateGridMap() {
        // Calculate grid dimensions based on resolution
        double area_width = area_max_x_ - area_min_x_;
        double area_height = area_max_y_ - area_min_y_;
        
        grid_width_ = static_cast<int>(std::ceil(area_width / grid_resolution_));
        grid_height_ = static_cast<int>(std::ceil(area_height / grid_resolution_));
        
        // Use the specified resolution as cell size
        double cell_width = grid_resolution_;
        double cell_height = grid_resolution_;
        
        grid_values_.clear();
        grid_values_.reserve(grid_width_ * grid_height_);
        
        // Calculate min/max for normalization
        double min_val = std::numeric_limits<double>::max();
        double max_val = std::numeric_limits<double>::lowest();
        
        // First pass: sample all points and find range
        std::vector<double> raw_values;
        raw_values.reserve(grid_width_ * grid_height_);
        
        for (int j = 0; j < grid_height_; ++j) {
            for (int i = 0; i < grid_width_; ++i) {
                double x = area_min_x_ + (i + 0.5) * cell_width;
                double y = area_min_y_ + (j + 0.5) * cell_height;
                double value = generator_->sample(x, y);
                raw_values.push_back(value);
                min_val = std::min(min_val, value);
                max_val = std::max(max_val, value);
            }
        }
        
        // Store normalized values
        double value_range = max_val - min_val;
        if (value_range < 1e-9) value_range = 1.0;  // Avoid division by zero
        
        for (double raw_val : raw_values) {
            double normalized = (raw_val - min_val) / value_range;
            grid_values_.push_back(normalized);
        }
        
        cell_width_ = cell_width;
        cell_height_ = cell_height;
        min_value_ = min_val;
        max_value_ = max_val;
    }

    void publishGrid() {
        if (grid_values_.empty()) {
            return;
        }

        auto grid_msg = std::make_shared<foxglove_msgs::msg::Grid>();
        
        // Set timestamp
        grid_msg->timestamp = this->get_clock()->now();
        grid_msg->frame_id = "map";
        
        // Set cell size
        grid_msg->cell_size.x = cell_width_;
        grid_msg->cell_size.y = cell_height_;
        
        // Set grid pose (origin at bottom-left)
        grid_msg->pose.position.x = area_min_x_;
        grid_msg->pose.position.y = area_min_y_;
        grid_msg->pose.position.z = 0.0;
        grid_msg->pose.orientation.w = 1.0;
        
        // Set grid dimensions
        grid_msg->column_count = grid_width_;
        
        // Set strides (5 bytes per cell: value + RGBA)
        grid_msg->row_stride = grid_width_ * 5;
        grid_msg->cell_stride = 5;
        
        // Define fields
        foxglove_msgs::msg::PackedElementField field_value;
        field_value.name = "value";
        field_value.offset = 0;
        field_value.type = foxglove_msgs::msg::PackedElementField::UINT8;
        
        foxglove_msgs::msg::PackedElementField field_red;
        field_red.name = "red";
        field_red.offset = 1;
        field_red.type = foxglove_msgs::msg::PackedElementField::UINT8;
        
        foxglove_msgs::msg::PackedElementField field_green;
        field_green.name = "green";
        field_green.offset = 2;
        field_green.type = foxglove_msgs::msg::PackedElementField::UINT8;
        
        foxglove_msgs::msg::PackedElementField field_blue;
        field_blue.name = "blue";
        field_blue.offset = 3;
        field_blue.type = foxglove_msgs::msg::PackedElementField::UINT8;
        
        foxglove_msgs::msg::PackedElementField field_alpha;
        field_alpha.name = "alpha";
        field_alpha.offset = 4;
        field_alpha.type = foxglove_msgs::msg::PackedElementField::UINT8;
        
        grid_msg->fields = {field_value, field_red, field_green, field_blue, field_alpha};
        
        // Prepare data buffer
        std::vector<uint8_t> grid_data;
        grid_data.reserve(grid_width_ * grid_height_ * 5);
        
        for (double normalized_val : grid_values_) {
            // Convert normalized value to 0-255
            uint8_t value = static_cast<uint8_t>(std::clamp(normalized_val * 255.0, 0.0, 255.0));
            
            // Color mapping: red for high values, blue for low values
            uint8_t red = value;
            uint8_t green = 25;
            uint8_t blue = 255 - value;
            uint8_t alpha = 255;  // Fully opaque for ground truth
            
            grid_data.push_back(value);
            grid_data.push_back(red);
            grid_data.push_back(green);
            grid_data.push_back(blue);
            grid_data.push_back(alpha);
        }
        
        grid_msg->data = grid_data;
        
        // Publish
        grid_publisher_->publish(*grid_msg);
    }

    rclcpp::Service<safe_scout_simulator::srv::SampleGroundTruth>::SharedPtr service_;
    rclcpp::Publisher<foxglove_msgs::msg::Grid>::SharedPtr grid_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::unique_ptr<DenseGroundTruthGenerator> generator_;
    size_t reported_num_gaussians_ {0};
    
    // Grid map data
    std::vector<double> grid_values_;
    int grid_width_ = 0;
    int grid_height_ = 0;
    double cell_width_ = 0.0;
    double cell_height_ = 0.0;
    double area_min_x_ = 0.0;
    double area_max_x_ = 0.0;
    double area_min_y_ = 0.0;
    double area_max_y_ = 0.0;
    double grid_resolution_ = 0.3;  // Default resolution in meters
    double min_value_ = 0.0;
    double max_value_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthServerNode>());
    rclcpp::shutdown();
    return 0;
}
