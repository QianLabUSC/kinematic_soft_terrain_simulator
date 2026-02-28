import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


# Read the CSV file, skipping the first two lines
df = pd.read_csv('scripts/soil_calib_scripts/GB_V0565_1_Tue_Jan_30_11_14_09_2024.csv', skiprows=2)


#plt.plot(df["toe_position_y"], df["toeforce_y"])
plt.plot(df["time"],df["toe_position_y"])
plt.show()

minTimeVal = 2.56
maxTimeVal = 9

minIndex = df[df['time'] > minTimeVal].index[0]
maxIndex = df[df['time'] > maxTimeVal].index[0]

time = df["time"][minIndex:maxIndex]
toePos = np.array(df["toe_position_y"][minIndex:maxIndex])
toePos = -(toePos - toePos[0])
toeForce = np.array(df["toeforce_y"][minIndex:maxIndex])
toeForce = toeForce-toeForce[0]

plt.plot(toePos, toeForce)
plt.show()


slope, intercept = np.polyfit(toePos, toeForce, 1)
print(slope, intercept)