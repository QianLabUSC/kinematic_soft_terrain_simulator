#include "sim_tools/VariableSoilParams.h"
#include "sim_tools/KDTree.h"
#include "sim_tools/GroundStiffness.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

VariableSoilParams::VariableSoilParams(std::string filename, double scalefactor) {
    readCSV(filename, scalefactor);

    if (!rclcpp::ok()) {
        std::cerr << "[VariableSoilParams] ROS2 is not initialized; "
                  << "soil service client disabled, using fallback stiffness model."
                  << std::endl;
        return;
    }

    static std::atomic<uint64_t> client_counter{0};
    const uint64_t client_id = client_counter.fetch_add(1);
    service_node_ = std::make_shared<rclcpp::Node>(
        "variable_soil_params_client_" + std::to_string(client_id));
    sample_ground_truth_client_ =
        service_node_->create_client<safe_scout_simulator::srv::SampleGroundTruth>(
            service_name_);
}

std::vector<GroundStiffness> VariableSoilParams::readCSV(const std::string& filename, double scalefactor) {
    std::ifstream file(filename);
    std::string line;
    double index = 0;

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue; // Skip empty lines
        }

        std::stringstream ss(line);
        std::string temp_x, temp_y, temp_stiffness;

        if (!std::getline(ss, temp_x, ',')) continue;
        if (!std::getline(ss, temp_y, ',')) continue;
        if (!std::getline(ss, temp_stiffness)) continue;

        try {
            GroundStiffness temp;
            temp.x = std::stod(temp_x);
            temp.y = std::stod(temp_y);
            temp.stiffness = std::stod(temp_stiffness) * scalefactor;

            stiffness_map.emplace_back(temp);
            tree.insert(temp);
            index++;
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid data at line " << index + 1 << ": " << e.what() << std::endl;
            continue; // Skip lines with invalid data
        }
    }

    return stiffness_map;
}

float VariableSoilParams::findClosest(std::vector<GroundStiffness> P, float x, float y) { 
    GroundStiffness query_point;
    query_point.x = x;
    query_point.y = y;

    GroundStiffness nearest = tree.nearestNeighbor(query_point);
    return nearest.stiffness;
} 

float VariableSoilParams::GetStiffnessAt(double x, double y) {
    return findClosest(stiffness_map, x, y);
}

float VariableSoilParams::dist(GroundStiffness p1, float x, float y) { 
    return sqrt((p1.x - x) * (p1.x - x) + (p1.y - y) * (p1.y - y)); 
} 

void VariableSoilParams::SetServiceQueryRobotCenter(double x_robot, double y_robot) {
    std::lock_guard<std::mutex> lock(query_pose_mutex_);
    service_query_robot_x_ = x_robot;
    service_query_robot_y_ = y_robot;
    use_robot_center_for_service_query_ = true;
}

int64_t VariableSoilParams::makeCacheKey(double x, double y, double resolution) {
    const int32_t qx = static_cast<int32_t>(std::llround(x / resolution));
    const int32_t qy = static_cast<int32_t>(std::llround(y / resolution));
    return (static_cast<int64_t>(qx) << 32) ^
           static_cast<uint32_t>(qy);
}

double VariableSoilParams::ServiceNormalizedToBekkerKphi(double normalized_value) {
    const double normalized = std::clamp(normalized_value, 0.0, 1.0);
    if (normalized <= 0.5) {
        // 0.0 -> 4.0e6, 0.5 -> 1.0e6
        const double t_safe = normalized / 0.5;
        return 4.0e6 - t_safe * 3.0e6;
    }

    // 0.5 -> 1.0e6, then rapidly decays toward 0.5e6
    const double t_unsafe = (normalized - 0.5) / 0.5;  // [0, 1]
    const double rapid_decay = std::exp(-12.0 * t_unsafe);
    return 0.2e6 + 0.8e6 * rapid_decay;
}

double VariableSoilParams::BekkerKphiToServiceNormalized(double bekker_kphi) {
    const double kphi = std::clamp(bekker_kphi, 0.5e6, 4.0e6);
    if (kphi >= 1.0e6) {
        // Inverse of linear branch:
        // kphi = 4.0e6 - 6.0e6 * normalized
        const double normalized = (4.0e6 - kphi) / 6.0e6;
        return std::clamp(normalized, 0.0, 0.5);
    }

    // Inverse of exponential branch:
    // kphi = 0.5e6 + 0.5e6 * exp(-12 * t), normalized = 0.5 + 0.5 * t
    const double ratio = std::clamp((kphi - 0.5e6) / 0.5e6, 1e-12, 1.0);
    const double t_unsafe = -std::log(ratio) / 12.0;
    const double normalized = 0.5 + 0.5 * t_unsafe;
    return std::clamp(normalized, 0.5, 1.0);
}

double VariableSoilParams::ComputeBekkerKphiAt(const ChVector3d& loc) {
    double sampled_stiffness = 0.0;
    double query_x = loc.x();
    double query_y = loc.y();
    {
        std::lock_guard<std::mutex> lock(query_pose_mutex_);
        if (use_robot_center_for_service_query_) {
            query_x = service_query_robot_x_;
            query_y = service_query_robot_y_;
        }
    }

    const bool got_service_value =
        queryStiffnessFromService(query_x, query_y, sampled_stiffness);
    if (got_service_value) {
        return ServiceNormalizedToBekkerKphi(sampled_stiffness);
    }

    // Legacy fallback path (kept intentionally):
    // Normalize coordinates from (-2,2), (-3,3) to (0,4), (0,6).
    const double scale_factor = 3.0;
    const double x = (loc.x() / scale_factor + 2.0);  // (-2,2) -> (0,4)
    const double y = (loc.y() / scale_factor + 3.0);  // (-3,3) -> (0,6)
    const double matrix_value = getMatrixValues(x, y);
    return matrix_value * 1e6;
}

bool VariableSoilParams::queryStiffnessFromService(double x, double y,
                                                   double& stiffness_out) {
    const int64_t key = makeCacheKey(x, y, cache_resolution_);
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        auto it = stiffness_cache_.find(key);
        if (it != stiffness_cache_.end()) {
            stiffness_out = it->second;
            return true;
        }
    }

    std::lock_guard<std::mutex> client_lock(client_mutex_);
    if (!sample_ground_truth_client_ || !service_node_) {
        return false;
    }

    if (!sample_ground_truth_client_->service_is_ready()) {
        if (!sample_ground_truth_client_->wait_for_service(
                std::chrono::milliseconds(50))) {
            if (!service_warned_unavailable_) {
                RCLCPP_WARN(service_node_->get_logger(),
                            "Service '%s' not available; using fallback "
                            "matrix stiffness model.",
                            service_name_.c_str());
                service_warned_unavailable_ = true;
            }
            return false;
        }
    }

    auto request =
        std::make_shared<safe_scout_simulator::srv::SampleGroundTruth::Request>();
    request->x = x;
    request->y = y;

    auto future = sample_ground_truth_client_->async_send_request(request);
    const auto status = rclcpp::spin_until_future_complete(
        service_node_, future, std::chrono::milliseconds(100));
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
        return false;
    }

    auto response = future.get();
    stiffness_out = response->value;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        stiffness_cache_[key] = stiffness_out;
    }
    return true;
}

// Matrix-based soil values: NO interpolation (discrete lookup only)
double VariableSoilParams::getMatrixValues(double x, double y) {
    // Base 6x4 stiffness matrix
    std::vector<std::vector<double>> matrix = {
        {4.168627199610553, 1.1639668053843606, 0.30710529885675586, 0.7411819632794852},  
        {4.932041832944858, 1.141251517039624, 0.297053272411554, 0.2773089503787785},  
        {4.461092501564501, 5.7528175636322985, 1.4108898440679434, 2.3547419484909445}, 
        {1.225502430363438, 4.037080618643726, 3.927288882222732, 2.4097770750500039},  
        {0.35158336758125375, 0.35430858539762444, 5.984094557388797, 4.867289332822818}, 
        {1.0330253761084038, 0.5566564093020394, 8.3833557993208, 4.156732259680446}     
    };
    
    // Clamp coordinates to valid range
    x = std::max(0.0, std::min(3.0, x));
    y = std::max(0.0, std::min(5.0, y));

    // Return discrete cell value by clamped floor indices
    int col = static_cast<int>(std::floor(std::max(0.0, std::min(3.0, x))));
    int row = static_cast<int>(std::floor(std::max(0.0, std::min(5.0, y))));
    row = std::max(0, std::min(5, row));
    col = std::max(0, std::min(3, col));
    return matrix[row][col];
}
//0,-550.0,-700.0
void VariableSoilParams::Set(const ChVector3d& loc,
                    double& Bekker_Kphi,
                    double& Bekker_Kc,
                    double& Bekker_n,
                    double& Mohr_cohesion,
                    double& Mohr_friction,
                    double& Janosi_shear,
                    double& elastic_K,
                    double& damping_R) {
    const double stiffness = ComputeBekkerKphiAt(loc);
    Bekker_Kphi = stiffness;


    bool default_params = true;
    if (default_params) {
        Bekker_Kc = -100e3;
        Bekker_n = 1.0;
        Mohr_cohesion = 1.6e3;
        Mohr_friction = 31.1;
        Janosi_shear = 1.2e-2;
        elastic_K = 8 * stiffness; 
        damping_R = 5e4; 
    } else {
        float max_scale = 1.5e4; 
        Bekker_n        = 1.1;      
        Mohr_cohesion   = 0.0;        
        Janosi_shear    = 0.005;         
        damping_R       = 2.4e4 * stiffness / max_scale;  

        if (Bekker_Kphi > 5000) Bekker_Kphi = 5000;
        if (Bekker_Kphi < 500)  Bekker_Kphi = 500;
        Bekker_Kc       = -700;

        double t = (Bekker_Kphi - 500) / (5000.0 - 500.0);

        Mohr_friction   = 30;        
        elastic_K       = 0.4e6 + t * (2e6 - 0.4e6);     
        damping_R       = 1600 + t * (8000 - 1600);     
    }
}
