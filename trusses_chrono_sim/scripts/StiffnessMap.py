import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class StiffnessMap:
    def __init__(self, csv_path, length, width, pixel_size):
        self.csv_path = csv_path
        self.length = length
        self.width = width
        self.pixel_size = pixel_size
        self.points = None
        self.stiffness_values = None

    def read_csv(self):
        # Read the CSV file, assuming columns are x, y, stiffness
        data = pd.read_csv(self.csv_path, header=None)
        self.points = data.iloc[:, 0:2].values
        self.stiffness_values = data.iloc[:, 2].values

    def generate_map(self):
        # Create a grid for visual representation, centered at (0, 0)
        x_coords = np.arange(-self.length / 2, self.length / 2, self.pixel_size)
        y_coords = np.arange(-self.width / 2, self.width / 2, self.pixel_size)
        xv, yv = np.meshgrid(x_coords, y_coords)

        # Initialize a grid to store the stiffness values
        stiffness_map = np.zeros(xv.shape)

        # Assign stiffness values based on nearest points (Voronoi diagram regions)
        for i in range(len(xv)):
            for j in range(len(yv)):
                distances = np.linalg.norm(self.points - np.array([xv[i, j], yv[i, j]]), axis=1)
                nearest_point_idx = np.argmin(distances)
                stiffness_map[i, j] = self.stiffness_values[nearest_point_idx]

        return stiffness_map

    def plot_stiffness_map(self, stiffness_map):
        # Plot the stiffness map as a heatmap
        plt.figure(figsize=(10, 8))
        plt.imshow(stiffness_map, extent=(-self.length / 2, self.length / 2, -self.width / 2, self.width / 2), origin='lower', cmap='pink', interpolation='nearest', aspect='equal')
        plt.colorbar(label='Stiffness')
        plt.title("Stiffness Heatmap")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.show()

    def save_stiffness_map(self, stiffness_map, filename_original, filename_flipped):
        # Plot the original stiffness map without axes and save it
        plt.figure(figsize=(10, 8))
        plt.imshow(stiffness_map, extent=(-self.length / 2, self.length / 2, -self.width / 2, self.width / 2), origin='lower', cmap='pink', interpolation='nearest', aspect='equal')
        plt.axis('off')  # Remove axes
        plt.savefig(filename_original, bbox_inches='tight', pad_inches=0)
        plt.close()

        # Flip the map horizontally (along the y-axis)
        flipped_map = np.flipud(stiffness_map)

        # Plot the flipped map without axes and save it
        plt.figure(figsize=(10, 8))
        plt.imshow(flipped_map, extent=(-self.length / 2, self.length / 2, -self.width / 2, self.width / 2), origin='lower', cmap='pink', interpolation='nearest', aspect='equal')
        plt.axis('off')  # Remove axes
        plt.savefig(filename_flipped, bbox_inches='tight', pad_inches=0)
        plt.close()

    def save_stiffness_csv(self, stiffness_map, output_csv_path):
        # Create x and y coordinates for the grid
        x_coords = np.arange(-self.length / 2, self.length / 2, self.pixel_size)
        y_coords = np.arange(-self.width / 2, self.width / 2, self.pixel_size)
        
        # Flatten the meshgrid and stiffness map to match x, y, stiffness format
        xv, yv = np.meshgrid(x_coords, y_coords)
        flat_x = np.round(xv.flatten(),4)
        flat_y = np.round(yv.flatten(),4)
        flat_stiffness = stiffness_map.flatten()

        # Combine x, y, and stiffness into a DataFrame
        data = {'x': flat_x, 'y': flat_y, 'stiffness': flat_stiffness}
        df = pd.DataFrame(data)

        # Save the DataFrame to a CSV file
        df.to_csv(output_csv_path, index=False, header=False)
        print(f"Stiffness map saved to {output_csv_path}")

if __name__ == "__main__":

    csv_path = "./scripts/Georgia.csv"
    length = 5  # Map length in meters
    width = 5  # Map width in meters
    pixel_size = 0.05  # Pixel size for the visual map

    # Create the stiffness map object
    stiffness_map = StiffnessMap(csv_path, length, width, pixel_size)
    
    # Read data from CSV
    stiffness_map.read_csv()
    # Generate stiffness map based on Voronoi regions
    stiffness_map_grid = stiffness_map.generate_map()

    stiffness_map.plot_stiffness_map(stiffness_map_grid)

    # Save the stiffness map and its flipped version
    stiffness_map.save_stiffness_map(stiffness_map_grid, './scripts/Georgia.png', './scripts/Georgia_flipped.png')

    stiffness_map.save_stiffness_csv(stiffness_map_grid, './scripts/stiffness_map_output.csv')
