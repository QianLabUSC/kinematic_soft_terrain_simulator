import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

class ChronoAnalyzer:
    def __init__(self, filepath, delimiter=' ', window_length=100, polyorder=1):
        # Load data
        self.df = pd.read_csv(filepath, delimiter=delimiter)
        
        # Extract columns
        self.time = self.df.iloc[:, 0].values
        self.force_x = self.df.iloc[:, 1].values
        self.force_y = self.df.iloc[:, 2].values
        self.force_z = self.df.iloc[:, 3].values
        self.probe_dist = self.df.iloc[:, 4].values
        self.probe_shear = None
        try:
            self.probe_shear = self.df.iloc[:, 5].values
        except IndexError:
            print("No shear data available")
        
        # Find the first non-zero index in force_y
        self.first_non_zero_index = np.nonzero(self.force_y)[0][0]
        
        # Slice the data from the first non-zero index
        self.time = self.time[self.first_non_zero_index:]
        self.force_y = self.force_y[self.first_non_zero_index:]
        self.force_z = self.force_z[self.first_non_zero_index:]
        self.probe_dist = self.probe_dist[self.first_non_zero_index:] - self.probe_dist[self.first_non_zero_index]
        if self.probe_shear is not None:
            self.probe_shear = self.probe_shear[self.first_non_zero_index:]
        
        # Apply Savitzky-Golay filter to smooth the data
        self.window_length = window_length
        self.polyorder = polyorder
        if(self.window_length == 0):
            self.force_y_filtered = self.force_y
            self.force_z_filtered = self.force_z
        else:
            self.force_y_filtered = savgol_filter(self.force_y, self.window_length, self.polyorder)
            self.force_z_filtered = savgol_filter(self.force_z, self.window_length, self.polyorder)
        
        # Calculate the slope and intercept for force_y_filtered vs probe_dist
        self.slope, self.intercept = np.polyfit(self.probe_dist, self.force_y_filtered, 1)

    
    def setup_plot(self):
        plt.figure(figsize=[14, 12])
        plt.rcParams.update({'font.size': 21})
        plt.rcParams['font.family'] = "Times New Roman"

        # Define a colorblind-friendly color palette
        colorblind_palette = ["#E15759", "#76B7B2", "#59A14F", "#EDC949", "#4E79A7", "#F28E2B", "#AF7AA1", "#FF9DA7", "#9C755F", "#BAB0AC"]

        # Set the color cycle to use the colorblind palette
        plt.rcParams['axes.prop_cycle'] = plt.cycler(color=colorblind_palette)
    
    def plot_force_vs_probe_dist(self):

        if self.probe_shear is None or np.isnan(self.probe_shear[0]):
            plt.subplot(1, 1, 1)
        else:
            plt.subplot(2, 1, 1)

        plt.plot(self.probe_dist, self.force_y_filtered,linewidth=6)
        plt.xlabel('Probe Displacement (m)')
        plt.ylabel('Force (N)')
        plt.title('Probe Vertical Displacement vs Force')
        plt.tight_layout()

    def plot_shear_vs_force(self):
        if self.probe_shear is not None and not np.isnan(self.probe_shear[0]):
            plt.subplot(2, 1, 2)
            plt.plot(self.probe_shear, self.force_z_filtered)
            plt.xlabel('Probe Shear Displacement (m)')
            plt.ylabel('Force (N)')
            plt.title('Filtered Probe Shear Displacement vs Force')
        
        plt.tight_layout()

        
    def get_slope_intercept(self):
        return self.slope, self.intercept
    

if(__name__ == '__main__'):
    analyzer = ChronoAnalyzer('./probe_output/output.dat',window_length=0)
    analyzer.setup_plot()
    analyzer.plot_force_vs_probe_dist()
    analyzer.plot_shear_vs_force()
    plt.show()
    slope, intercept = analyzer.get_slope_intercept()