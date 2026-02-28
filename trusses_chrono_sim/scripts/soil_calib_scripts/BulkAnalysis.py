from CSVAnalysis import CSVAnalysis
from ChronoAnalyzer import ChronoAnalyzer
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter

class BulkAnalysis:
    def __init__(self, file_names):
        self.data = []
        for file_name in file_names:
            self.data.append(CSVAnalysis(file_name))
            self.data[-1].set_offset()
        
        self.forces = []
        self.positions = []
        self.times = []

        # Collect all the time, force, and position data
        for i in range(len(file_names)):
            self.times.append(self.data[i].time)
            self.forces.append(self.data[i].toe_force)
            self.positions.append(self.data[i].toe_pos)

        self.common_time = np.linspace(min(min(t) for t in self.times), max(max(t) for t in self.times), num=6000)

        self.total_force = np.zeros_like(self.common_time)
        self.total_pos = np.zeros_like(self.common_time)

        # Interpolate and sum all forces and positions over the common time base
        for i in range(len(file_names)):
            interp_force = interp1d(self.times[i], self.forces[i], kind='linear', fill_value="extrapolate")(self.common_time)
            interp_pos = interp1d(self.times[i], self.positions[i], kind='linear', fill_value="extrapolate")(self.common_time)
            self.total_force += interp_force
            self.total_pos += interp_pos

        # Calculate the average force and position
        self.avg_force = self.total_force / len(file_names)
        self.avg_pressure = self.total_force / len(file_names) / .0001266
        self.avg_pos = self.total_pos / len(file_names)

        self.init_index = np.where(self.avg_pos > 0.02)[0][0]

        self.avg_force = self.avg_force[self.init_index:]
        self.avg_pressure = self.avg_pressure[self.init_index:]
        self.avg_pos = self.avg_pos[self.init_index:]

        self.avg_pos = self.avg_pos - self.avg_pos[0]+.0001

        #self.avg_force = savgol_filter(self.avg_force, 600, 2)
        #self.avg_pressure = savgol_filter(self.avg_pressure, 600, 2)
        #self.avg_pos = savgol_filter(self.avg_pos, 600, 2)

        self.slope, self.intercept = np.polyfit(self.avg_pos, self.avg_force, 1)

        self.p2 = self.avg_pressure**2
        self.p_sum = np.sum(self.avg_pressure)
        self.p2_sum = np.sum(self.p2)

        self.ln_p = np.log(self.avg_pressure)
        self.ln_z = np.log(self.avg_pos)

        self.n = (self.p2_sum*np.sum(self.p2*self.ln_p*self.ln_z)-np.sum(self.p2*self.ln_p)*np.sum(self.p2*self.ln_z)) / (self.p2_sum*np.sum(self.p2*(self.ln_z**2))-(np.sum(self.p2*self.ln_z))**2)

        self.ln_keq = (np.sum(self.p2*self.ln_p)-self.n*np.sum(self.p2*self.ln_z)) / (self.p2_sum)
        self.keq = np.exp(self.ln_keq)

        self.chrono_probe = ChronoAnalyzer('./probe_output/output.dat',window_length=0)

    def plot_files(self):
        for i in range(len(self.data)):
            index = np.where(self.data[i].toe_pos > 0.02)[0][0]
            plt.plot(self.data[i].toe_pos[index:]-.02, self.data[i].toe_force[index:])
    
    def plot_chrono(self):
        self.chrono_probe.plot_force_vs_probe_dist()

    def plot_avg(self):
        plt.plot(self.avg_pos, self.avg_force, linewidth=6)