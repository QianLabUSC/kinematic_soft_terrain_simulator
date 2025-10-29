#include <memory>
#include <vector>
#include <random>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "safe_scout_simulator/srv/sample_ground_truth.hpp"

struct Gaussian2D {
    double mean_x;
    double mean_y;
    double std_dev;
    double amplitude;
};

class DenseGroundTruthGenerator {
public:
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
            double dx = x - gaussian.mean_x;
            double dy = y - gaussian.mean_y;
            double dist_sq = dx * dx + dy * dy;
            double variance = gaussian.std_dev * gaussian.std_dev;

            value += gaussian.amplitude * std::exp(-dist_sq / (2.0 * variance));
        }

        return value;
    }

private:
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

        gaussians_.reserve(num_gaussians);
        for (int i = 0; i < num_gaussians; ++i) {
            Gaussian2D g;
            g.mean_x = dis_x(gen);
            g.mean_y = dis_y(gen);
            g.std_dev = dis_std(gen);
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

        // Get parameters
        double area_min_x = this->get_parameter("area_min_x").as_double();
        double area_max_x = this->get_parameter("area_max_x").as_double();
        double area_min_y = this->get_parameter("area_min_y").as_double();
        double area_max_y = this->get_parameter("area_max_y").as_double();
        int num_gaussians = this->get_parameter("num_gaussians").as_int();
        double lipschitz_constant = this->get_parameter("lipschitz_constant").as_double();
        int random_seed = this->get_parameter("random_seed").as_int();

        // Initialize ground truth generator
        generator_ = std::make_unique<DenseGroundTruthGenerator>(
            area_min_x, area_max_x, area_min_y, area_max_y,
            num_gaussians, lipschitz_constant, random_seed);

        // Create service
        service_ = this->create_service<safe_scout_simulator::srv::SampleGroundTruth>(
            "sample_ground_truth",
            std::bind(&GroundTruthServerNode::handleSampleRequest, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Ground Truth Server initialized");
        RCLCPP_INFO(this->get_logger(), "Area: [%.2f, %.2f] x [%.2f, %.2f]",
                    area_min_x, area_max_x, area_min_y, area_max_y);
        RCLCPP_INFO(this->get_logger(), "Number of Gaussians: %d", num_gaussians);
        RCLCPP_INFO(this->get_logger(), "Lipschitz constant: %.2f", lipschitz_constant);
        if (random_seed >= 0) {
            RCLCPP_INFO(this->get_logger(), "Random seed: %d (deterministic)", random_seed);
        } else {
            RCLCPP_INFO(this->get_logger(), "Random seed: random_device (non-deterministic)");
        }
    }

private:
    void handleSampleRequest(
        const std::shared_ptr<safe_scout_simulator::srv::SampleGroundTruth::Request> request,
        std::shared_ptr<safe_scout_simulator::srv::SampleGroundTruth::Response> response) {

        response->value = generator_->sample(request->x, request->y);

        RCLCPP_DEBUG(this->get_logger(), "Sampled (%.2f, %.2f) = %.4f",
                     request->x, request->y, response->value);
    }

    rclcpp::Service<safe_scout_simulator::srv::SampleGroundTruth>::SharedPtr service_;
    std::unique_ptr<DenseGroundTruthGenerator> generator_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthServerNode>());
    rclcpp::shutdown();
    return 0;
}
