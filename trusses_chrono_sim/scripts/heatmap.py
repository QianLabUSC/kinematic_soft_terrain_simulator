import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
import pandas as pd
import os

class Map:
    def __init__(self, length, width, resolution, ground_stiffness_range=None):
        self.length = length - resolution
        self.width = width - resolution
        self.resolution = resolution
        self.ground_stiffness_range = ground_stiffness_range or (0, 1)
        self.grid = None
        self.high_res_grid = None

    def generate_grid(self, worse_resolution_factor):
        worse_resolution = self.resolution * worse_resolution_factor
        x = np.arange(0, self.length + worse_resolution, worse_resolution)
        y = np.arange(0, self.width + worse_resolution, worse_resolution)
        xv, yv = np.meshgrid(x, y)
        xv = xv.flatten()
        yv = yv.flatten()

        # Generate random ground stiffness values within the specified range
        ground_stiffness = np.random.uniform(self.ground_stiffness_range[0], self.ground_stiffness_range[1], size=xv.shape)

        ground_stiffness = []
        
        for j in range(0,len(y)):
            init_stiffness = self.ground_stiffness_range[0]
            for i in range(0,len(x)):
                init_stiffness += (self.ground_stiffness_range[1]-self.ground_stiffness_range[0])/len(x)
                ground_stiffness.append(init_stiffness)


        self.grid = np.column_stack((yv, xv, ground_stiffness))
        
    def interpolate_grid(self):
        if self.grid is None:
            raise ValueError("Worse-resolution grid has not been generated yet.")
        
        x_high_res = np.arange(0, self.length + self.resolution, self.resolution)
        y_high_res = np.arange(0, self.width + self.resolution, self.resolution)
        xv_high_res, yv_high_res = np.meshgrid(x_high_res, y_high_res)

        # Interpolate ground stiffness values to the high-resolution grid
        ground_stiffness_high_res = griddata(
            (self.grid[:, 0], self.grid[:, 1]), self.grid[:, 2],
            (xv_high_res, yv_high_res), method='linear'
        )

        self.high_res_grid = np.column_stack((xv_high_res.flatten(), yv_high_res.flatten(), ground_stiffness_high_res.flatten()))

    def get_high_res_grid(self):
        if self.high_res_grid is None:
            raise ValueError("High-resolution grid has not been generated yet.")
        
        x_centers = self.high_res_grid[:, 0] + self.resolution / 2
        y_centers = self.high_res_grid[:, 1] + self.resolution / 2
        
        initial_grid = np.column_stack((x_centers, y_centers, self.high_res_grid[:, 2]))

        center_x = (self.length + self.resolution) / 2
        center_y = (self.width + self.resolution) / 2

        initial_grid[:, 0] -= center_x
        initial_grid[:, 1] -= center_y

        return np.around(initial_grid, 4)

    def save_heatmap(self):
        if self.high_res_grid is None:
            raise ValueError("High-resolution grid has not been generated yet.")
        
        heatmap_path = "./scripts/Map.png"
        if os.path.exists(heatmap_path):
            os.remove(heatmap_path)
        
        x = np.arange(0, self.length + self.resolution, self.resolution)
        y = np.arange(0, self.width + self.resolution, self.resolution)
        grid_shape = (len(y), len(x))
        ground_stiffness = self.high_res_grid[:, 2].reshape(grid_shape)
      
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.imshow(ground_stiffness, extent=(0, self.length, 0, self.width), origin='lower', cmap='pink', aspect='equal')

        # Remove axes and border
        ax.axis('off')
   
        plt.savefig(heatmap_path, format='png', bbox_inches='tight', pad_inches=0)

        #plt.invert_xaxis()
        ax.invert_yaxis()

        plt.savefig("./scripts/Map_flipped.png", format='png', bbox_inches='tight', pad_inches=0)


    def plot_heatmap(self, resolution='high'):
        if resolution == 'high':
            if self.high_res_grid is None:
                raise ValueError("High-resolution grid has not been generated yet.")
            x = np.arange(0, self.length + self.resolution, self.resolution)
            y = np.arange(0, self.width + self.resolution, self.resolution)
            ground_stiffness = self.high_res_grid[:, 2].reshape((len(y), len(x)))
        elif resolution == 'low':
            if self.grid is None:
                raise ValueError("Low-resolution grid has not been generated yet.")
            low_res = int(np.sqrt(self.grid.shape[0]))
            x = np.linspace(0, self.length, low_res)
            y = np.linspace(0, self.width, low_res)
            xv, yv = np.meshgrid(x, y)
            ground_stiffness = self.grid[:, 2].reshape((low_res, low_res))
        else:
            raise ValueError("Resolution must be 'high' or 'low'.")

        plt.figure(figsize=(10, 8))
        plt.imshow(ground_stiffness, extent=(0, self.length, 0, self.width), origin='lower', cmap='pink')
        plt.colorbar(label='Ground Stiffness (N/m^2)')
        plt.xlabel('Length (m)')
        plt.ylabel('Width (m)')
        plt.title(f'Ground Stiffness Heatmap ({resolution}-resolution)')
        plt.show()

if __name__ == "__main__":
    length = 2
    width = 2
    resolution = .05
    map_object = Map(length, width, resolution, (2000000, 5000000)) #100, 3000000

    # Generate worse-resolution grid
    worse_resolution_factor = 1
    map_object.generate_grid(worse_resolution_factor)

    # Interpolate to high-resolution grid
    map_object.interpolate_grid()

    map_object.plot_heatmap()
    map_object.save_heatmap()

    high_res_grid = map_object.get_high_res_grid()

    csv_path = "./scripts/map.csv"
    if os.path.exists(csv_path):
        os.remove(csv_path)

    # Write the DataFrame to a temporary CSV file
    temp_csv_path = "./scripts/temp_map.csv"
    df = pd.DataFrame(high_res_grid)
    df.to_csv(temp_csv_path, header=None, index=False)

    # Read the contents of the temporary CSV file and remove the last newline
    with open(temp_csv_path, 'r') as temp_file:
        lines = temp_file.readlines()
    with open(csv_path, 'w') as final_file:
        final_file.writelines(lines[:-1] + [lines[-1].rstrip('\n')])

    # Remove the temporary file
    os.remove(temp_csv_path)