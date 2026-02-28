#!/usr/bin/env python3

import sys
import warnings
warnings.filterwarnings('ignore')

import rclpy
from rclpy.node import Node
from safe_scout_simulator.srv import SampleGroundTruth

# Import with compatibility handling
try:
    import numpy as np
except ImportError:
    print("ERROR: numpy is not installed. Install with: sudo apt install python3-numpy")
    sys.exit(1)

try:
    from scipy.interpolate import Rbf
except ImportError:
    print("ERROR: scipy is not installed. Install with: sudo apt install python3-scipy")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use('Agg')  # Use non-interactive backend for compatibility
    import matplotlib.pyplot as plt
    from matplotlib import cm
except ImportError:
    print("ERROR: matplotlib is not installed. Install with: sudo apt install python3-matplotlib")
    sys.exit(1)


class RBFVisualization(Node):
    def __init__(self):
        super().__init__('rbf_visualization')

        # Declare parameters
        self.declare_parameter('num_training_points', 50)
        self.declare_parameter('grid_resolution', 50)
        self.declare_parameter('area_min_x', 0.0)
        self.declare_parameter('area_max_x', 100.0)
        self.declare_parameter('area_min_y', 0.0)
        self.declare_parameter('area_max_y', 100.0)

        # Get parameters
        self.num_training = self.get_parameter('num_training_points').value
        self.grid_res = self.get_parameter('grid_resolution').value
        self.area_min_x = self.get_parameter('area_min_x').value
        self.area_max_x = self.get_parameter('area_max_x').value
        self.area_min_y = self.get_parameter('area_min_y').value
        self.area_max_y = self.get_parameter('area_max_y').value

        # Create service client
        self.client = self.create_client(SampleGroundTruth, 'sample_ground_truth')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ground truth service...')

        self.get_logger().info('Ground truth service connected!')

        # Run visualization
        self.run_rbf_visualization()

    def sample_ground_truth(self, x, y):
        """Sample a point from the ground truth service."""
        request = SampleGroundTruth.Request()
        request.x = float(x)
        request.y = float(y)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().value
        else:
            self.get_logger().error('Service call failed')
            return 0.0

    def run_rbf_visualization(self):
        self.get_logger().info(f'Collecting {self.num_training} random training samples...')

        # Collect random training data
        np.random.seed(42)
        X_train = np.random.uniform(
            low=[self.area_min_x, self.area_min_y],
            high=[self.area_max_x, self.area_max_y],
            size=(self.num_training, 2)
        )

        y_train = np.array([self.sample_ground_truth(x, y) for x, y in X_train])

        self.get_logger().info('Training RBF interpolator...')

        # Train RBF interpolator using a smooth multiquadric radial basis
        # Using 'multiquadric' function which provides smooth interpolation
        rbf_interpolator = Rbf(X_train[:, 0], X_train[:, 1], y_train,
                               function='multiquadric', smooth=0.1)

        self.get_logger().info('Creating test grid for evaluation...')

        # Create test grid
        x_test = np.linspace(self.area_min_x, self.area_max_x, self.grid_res)
        y_test = np.linspace(self.area_min_y, self.area_max_y, self.grid_res)
        X_grid, Y_grid = np.meshgrid(x_test, y_test)
        X_test = np.column_stack([X_grid.ravel(), Y_grid.ravel()])

        self.get_logger().info(f'Sampling {len(X_test)} ground truth points...')

        # Get ground truth values
        y_true = np.array([self.sample_ground_truth(x, y) for x, y in X_test])

        self.get_logger().info('Computing RBF predictions...')

        # Get RBF predictions
        y_pred = rbf_interpolator(X_test[:, 0], X_test[:, 1])

        # Reshape for plotting
        y_true_grid = y_true.reshape(X_grid.shape)
        y_pred_grid = y_pred.reshape(X_grid.shape)

        # Calculate accuracy metrics
        mse = np.mean((y_true - y_pred) ** 2)
        rmse = np.sqrt(mse)
        mae = np.mean(np.abs(y_true - y_pred))
        max_error = np.max(np.abs(y_true - y_pred))
        r2 = 1 - (np.sum((y_true - y_pred) ** 2) / np.sum((y_true - np.mean(y_true)) ** 2))

        self.get_logger().info('=== Accuracy Metrics ===')
        self.get_logger().info(f'RMSE: {rmse:.4f}')
        self.get_logger().info(f'MAE: {mae:.4f}')
        self.get_logger().info(f'Max Error: {max_error:.4f}')
        self.get_logger().info(f'R² Score: {r2:.4f}')

        # Create visualization
        self.get_logger().info('Generating visualization...')
        fig = plt.figure(figsize=(18, 12))

        # Ground Truth
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        surf1 = ax1.plot_surface(X_grid, Y_grid, y_true_grid, cmap=cm.viridis, alpha=0.8)
        ax1.scatter(X_train[:, 0], X_train[:, 1], y_train, c='red', s=50, marker='o', label='Training samples')
        ax1.set_title('Ground Truth', fontsize=14, fontweight='bold')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Value')
        ax1.legend()
        fig.colorbar(surf1, ax=ax1, shrink=0.5)

        # RBF Prediction
        ax2 = fig.add_subplot(2, 3, 2, projection='3d')
        surf2 = ax2.plot_surface(X_grid, Y_grid, y_pred_grid, cmap=cm.viridis, alpha=0.8)
        ax2.scatter(X_train[:, 0], X_train[:, 1], y_train, c='red', s=50, marker='o', label='Training samples')
        ax2.set_title('RBF Interpolation', fontsize=14, fontweight='bold')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Value')
        ax2.legend()
        fig.colorbar(surf2, ax=ax2, shrink=0.5)

        # Error
        error_grid = np.abs(y_true_grid - y_pred_grid)
        ax3 = fig.add_subplot(2, 3, 3, projection='3d')
        surf3 = ax3.plot_surface(X_grid, Y_grid, error_grid, cmap=cm.Reds, alpha=0.8)
        ax3.set_title('Absolute Error', fontsize=14, fontweight='bold')
        ax3.set_xlabel('X')
        ax3.set_ylabel('Y')
        ax3.set_zlabel('|Error|')
        fig.colorbar(surf3, ax=ax3, shrink=0.5)

        # 2D Heatmaps
        ax4 = fig.add_subplot(2, 3, 4)
        im4 = ax4.contourf(X_grid, Y_grid, y_true_grid, levels=20, cmap=cm.viridis)
        ax4.scatter(X_train[:, 0], X_train[:, 1], c='red', s=30, marker='x', linewidths=2)
        ax4.set_title('Ground Truth (2D)', fontsize=14, fontweight='bold')
        ax4.set_xlabel('X')
        ax4.set_ylabel('Y')
        fig.colorbar(im4, ax=ax4)

        ax5 = fig.add_subplot(2, 3, 5)
        im5 = ax5.contourf(X_grid, Y_grid, y_pred_grid, levels=20, cmap=cm.viridis)
        ax5.scatter(X_train[:, 0], X_train[:, 1], c='red', s=30, marker='x', linewidths=2)
        ax5.set_title('RBF Interpolation (2D)', fontsize=14, fontweight='bold')
        ax5.set_xlabel('X')
        ax5.set_ylabel('Y')
        fig.colorbar(im5, ax=ax5)

        ax6 = fig.add_subplot(2, 3, 6)
        im6 = ax6.contourf(X_grid, Y_grid, error_grid, levels=20, cmap=cm.Reds)
        ax6.set_title('Absolute Error (2D)', fontsize=14, fontweight='bold')
        ax6.set_xlabel('X')
        ax6.set_ylabel('Y')
        fig.colorbar(im6, ax=ax6)

        # Add text box with metrics
        textstr = f'Training Samples: {self.num_training}\n'
        textstr += f'RMSE: {rmse:.4f}\n'
        textstr += f'MAE: {mae:.4f}\n'
        textstr += f'Max Error: {max_error:.4f}\n'
        textstr += f'R² Score: {r2:.4f}'

        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        fig.text(0.5, 0.02, textstr, fontsize=12, verticalalignment='bottom',
                 horizontalalignment='center', bbox=props)

        plt.tight_layout(rect=[0, 0.05, 1, 1])
        plt.savefig('/tmp/rbf_ground_truth_visualization.png', dpi=150, bbox_inches='tight')
        self.get_logger().info('Visualization saved to /tmp/rbf_ground_truth_visualization.png')
        self.get_logger().info('View with: xdg-open /tmp/rbf_ground_truth_visualization.png')
        plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = RBFVisualization()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
