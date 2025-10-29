# Safe Scout Simulator

A ROS 2 Humble package that bundles the simulation utilities used by the safe scouting stack. It provides:

- A configurable ground-truth service that synthesizes dense scalar fields
- A spatial measurement publisher that samples the ground-truth service
- A 2D fully actuated drive simulator that publishes robot pose and TF
- Launch files to compose the simulation nodes for quick experiments

## Package Structure

```
./
├── CMakeLists.txt
├── config/
│   └── ground_truth_params.yaml
├── launch/
│   ├── ground_truth_server.launch.py
│   ├── rbf_demo.launch.py
│   └── server_with_spatial_publisher.launch.py
├── package.xml
├── requirements.txt
├── scripts/
│   ├── drive_sim
│   ├── rbf_visualization.py
│   └── spatial_measurement_publisher.py
├── src/
│   └── ground_truth_server.cpp
└── srv/
    └── SampleGroundTruth.srv
```

## Installation

### Python Dependencies (Ubuntu 22.04)

Install the required numerical libraries from the Ubuntu repositories:

```bash
sudo apt update
sudo apt install python3-numpy python3-matplotlib python3-scipy
```

### Build

```bash
cd <your_workspace>
colcon build --packages-select safe_scout_simulator
source install/setup.bash
```

## Configuration

Edit `config/ground_truth_params.yaml` to tune the simulation:

- `area_min_x`, `area_max_x`, `area_min_y`, `area_max_y`: bounds of the rectangular domain.
- `num_gaussians`: number of Gaussian components composing the scalar field.
- `random_seed`: deterministic seed (`-1` chooses a non-deterministic seed).
- `lipschitz_constant`: controls how quickly the field varies.
- `spatial_measurement_publisher` parameters allow topic remapping, noise injection, and service selection.

## Running the Nodes

Launch the ground-truth service on its own:

```bash
ros2 launch safe_scout_simulator ground_truth_server.launch.py
```

Call the service from the command line:

```bash
ros2 service call /sample_ground_truth safe_scout_simulator/srv/SampleGroundTruth "{x: 50.0, y: 50.0}"
```

Run the spatial measurement publisher alongside the server:

```bash
ros2 launch safe_scout_simulator server_with_spatial_publisher.launch.py
```

Start the drive simulator (publishes `spirit/current_pose` and TF):

```bash
ros2 run safe_scout_simulator drive_sim
```

## Python Client Example

```python
import rclpy
from rclpy.node import Node
from safe_scout_simulator.srv import SampleGroundTruth


class GroundTruthClient(Node):
    def __init__(self):
        super().__init__("ground_truth_client")
        self.client = self.create_client(SampleGroundTruth, "sample_ground_truth")

    def sample_point(self, x, y):
        request = SampleGroundTruth.Request()
        request.x = x
        request.y = y

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().value
```

## RBF Interpolation Demo

Visualize how well radial basis functions recover the synthetic field:

```bash
ros2 launch safe_scout_simulator rbf_demo.launch.py
```

You can also run the visualization node directly:

```bash
ros2 run safe_scout_simulator rbf_visualization.py --ros-args \
  -p num_training_points:=100 \
  -p grid_resolution:=80
```

The demo stores the resulting plot at `/tmp/rbf_ground_truth_visualization.png`.

## How It Works

1. The ground-truth server samples a configurable amount of Gaussian components to synthesize a scalar field.
2. The spatial measurement publisher queries the service and produces `SpatialMeasurement` messages, optionally adding noise.
3. The 2D drive simulator integrates velocity commands (`spirit/ghost_trot_control`) to publish pose and broadcast TF.

These utilities are designed to be launched together with the safe scouting stack but can also be run independently for testing and visualization.
