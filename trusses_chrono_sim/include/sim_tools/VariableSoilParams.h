#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "sim_tools/KDTree.h"
#include "sim_tools/GroundStiffness.h"
#include "rclcpp/rclcpp.hpp"
#include "safe_scout_simulator/srv/sample_ground_truth.hpp"
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
using namespace chrono;

#ifndef VARIABLESOILPARAMS_H
#define VARIABLESOILPARAMS_H

class VariableSoilParams : public vehicle::SCMTerrain::SoilParametersCallback{
public:
//put this in KDTree.h
/*
struct GroundStiffness {
    double x;
    double y;
    double stiffness;
};
*/
std::vector<GroundStiffness> stiffness_map;
KDTree tree;

VariableSoilParams(std::string filename, double scalefactor);
std::vector<GroundStiffness> readCSV(const std::string& filename, double scalefactor);
//KDTree& tree);
float findClosest(std::vector<GroundStiffness> P, float x, float y);
float GetStiffnessAt(double x, double y);
float dist(GroundStiffness p1, float x, float y);
void SetServiceQueryRobotCenter(double x_robot, double y_robot);

// New functions for coordinate mapping
double getMatrixValues(double x, double y);  // x: 0-4, y: 0-6, returns single value
bool queryStiffnessFromService(double x, double y, double& stiffness_out);
static int64_t makeCacheKey(double x, double y, double resolution);
static double ServiceNormalizedToBekkerKphi(double normalized_value);
static double BekkerKphiToServiceNormalized(double bekker_kphi);
double ComputeBekkerKphiAt(const ChVector3d& loc);

virtual void Set(const ChVector3d& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R);
private:
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::Client<safe_scout_simulator::srv::SampleGroundTruth>::SharedPtr
      sample_ground_truth_client_;
  std::string service_name_{"sample_ground_truth"};
  std::unordered_map<int64_t, double> stiffness_cache_;
  std::mutex cache_mutex_;
  std::mutex client_mutex_;
  bool service_warned_unavailable_{false};
  const double cache_resolution_{0.05};
  double service_query_robot_x_{0.0};
  double service_query_robot_y_{0.0};
  bool use_robot_center_for_service_query_{false};
  std::mutex query_pose_mutex_;

};

#endif
