from CSVAnalysis import CSVAnalysis
from BulkAnalysis import BulkAnalysis
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

file_names_stiff = ["GB_V0625new_1_Thu_Feb_22_16_41_23_2024.csv",
                    "GB_V0625new_2_Thu_Feb_22_16_42_53_2024.csv",
                    "GB_V0625_1_Mon_Feb_19_19_58_21_2024.csv",
                    "GB_V0625_2_Mon_Feb_19_20_00_23_2024.csv"]

file_names_soft = ["GB_V0565_1_Tue_Jan_30_11_14_09_2024.csv", 
                   "GB_V0565_2_Tue_Jan_30_11_19_52_2024.csv",
                   "GB_V0565_3_Tue_Jan_30_11_25_43_2024.csv",
                   "GB_V0565_4_Tue_Jan_30_11_30_46_2024.csv"]

bulk = BulkAnalysis(file_names_stiff)
bulk.chrono_probe.setup_plot()
bulk.plot_files()
bulk.plot_avg()
bulk.plot_chrono()
plt.legend(["Test 1", "Test 2", "Test 3", "Test 4", "Test Average","Chrono Probe"])
plt.show()

print(bulk.n)
print(bulk.keq)
