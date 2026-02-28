#!/bin/bash

# Ask for the step size used in Chrono simulation
echo "Enter the step size used in the Chrono simulation (e.g., .004 or 0.004):"
read step_size

# Check if input is valid (a positive number, allowing formats like .004 or 0.004)
if [[ ! $step_size =~ ^[0-9]*([.][0-9]+)?$ ]] || [[ $(echo "$step_size <= 0" | bc) -ne 0 ]]; then
  echo "Invalid step size. Please enter a positive number (e.g., .004 or 0.004)."
  exit 1
fi

# Calculate the frame rate (FPS) as 1 / step_size
frame_rate=$(echo "1 / $step_size" | bc -l)

# Set the output video name
output_video="output.mp4"

# Use ffmpeg to generate the video with correct timing
ffmpeg -framerate $frame_rate -i frame_%d.png -c:v libx264 -r 60 -pix_fmt yuv420p $output_video

echo "Video created: $output_video"
