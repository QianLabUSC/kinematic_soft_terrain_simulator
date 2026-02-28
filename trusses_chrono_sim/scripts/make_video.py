import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob
import time

#test = "robot1_success_region2_3_chrono"
#test = "robot1_failure_chrono"
test = "simple_success"
frames = sorted(glob.glob("frames/" + test + "/frame*.png"))

plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()

for frame_path in frames:
    img = mpimg.imread(frame_path)
    ax.imshow(img)
    ax.axis('off')
    plt.draw()
    plt.pause(0.00008)
    ax.clear()

plt.ioff()  # Turn off interactive mode
plt.close()
