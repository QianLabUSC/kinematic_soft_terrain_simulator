import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class CSVAnalysis:
    def __init__(self, csv_file, skip_rows=2):
        self.df = pd.read_csv("scripts/soil_calib_scripts/"+csv_file, skiprows=skip_rows)
        self.min_index = None
        self.max_index = None
        self.time = self.df["time"]
        self.toe_pos = self.df["toe_position_y"]
        self.toe_force = self.df["toeforce_y"]
    
    def offset_helper(self):
        plt.plot(self.time, self.toe_pos)
        plt.show()
    
    def set_offset(self,min_time_val=2.56,max_time_val=9):
        self.min_index = self.df[self.df['time'] > min_time_val].index[0]
        self.max_index = self.df[self.df['time'] > max_time_val].index[0]
        self.time = np.array(self.time[self.min_index:self.max_index])
        self.toe_pos = np.array(self.toe_pos[self.min_index:self.max_index])
        self.toe_force = np.array(self.toe_force[self.min_index:self.max_index])
        self.toe_pos = -(self.toe_pos - self.toe_pos[0])
        self.toe_force = self.toe_force - self.toe_force[0]

    def plot_force_displacement(self):
        plt.plot(self.toe_pos, self.toe_force)
    
    def get_slope(self):
        slope, intercept = np.polyfit(self.toe_pos, self.toe_force, 1)
        return slope, intercept
    
