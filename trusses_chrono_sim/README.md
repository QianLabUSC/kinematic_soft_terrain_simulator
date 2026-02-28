# What is this repository?
This repository is a [ROS2 Humble](https://docs.ros.org/en/humble/index.html) C++ package for simulating deformable terrain locomotion of legged, and wheeled systems using [Project Chrono](https://projectchrono.org). Specifically, this package utilizes Project Chrono's [SCM deformable terrain](https://api.projectchrono.org/7.0.0/vehicle_terrain.html#vehicle_terrain_scm) and also custom URDF models from Ghost Robotics for Spirit, and ones built by the TRUSSES group for a wheeled rover and a RHex-like robot. This repository's goal is to provide realistic simulations of robot interactions with both the lunar terrain and truss systems, while utilizing the same controllers and planners built for the physical systems they represent.

# Installation
Prerequesits to being able to build this ROS2 package on your machine are: ROS2 Humble and Project Chrono (currently the dev branch version). Install instructions for both of these can be found at the following [Google Docs file](https://docs.google.com/document/d/1gAzgMTsca_4tpOX5p5CMQO-JFgw7EE_JgvhAZQybD40/edit?usp=sharing) in the TRUSSES Google Drive. Follow instructions up until, but not including, creating your own ROS2 Chrono package. That is not necessary, as you will be installing this one in it's place. Now, let's get started installing this package assuming you do not have a ROS2 workspace already that you are installing into.
```
cd ~
mkdir ros2_ws
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
cd ~/ros2_ws/src
git clone git@github.com:TRUSSES/chrono_sim.git
cd ~/ros2_ws
colcon build
```
And that's it! If it is the case that you already have a ROS2 workspace you want to install this package in, then follow these instructions:
```
cd PATH/TO/YOUR/WORKSPACE
cd src
git clone git@github.com:TRUSSES/chrono_sim.git
cd ..
colcon build
```

# Usage
Currently, this package will auto-populate a `bin` directory inside the `chrono_sim` package with executables that will launch the simulation that corresponds to it's respective `.cpp` main file. You should be able to run any of these by first- ensuring you have sourced both ROS2 with ```source /opt/ros/humble/setup.bash``` AND your specific ROS2 workspace with 
```
source ~/ros2_ws/install/setup.bash
``` 
then second- by running 
```
cd ~/ros2_ws/src/chrono_sim/bin
```
and 
```
./spirit_example
```
or any other file name inside `bin`. Alternatively, you should also be able to use the command 
```
ros2 run chrono_sim spirit_example
```
from anywhere after sourcing your workspace. 

## Examples
### Spirit Example
Running the `spirit_example` executable will initialize the simulation, handling interrupts  (CTRL-C) cleanly, setting gravity, choosing thread count for the collision system, choosing an integration solver, and finally choosing an integration step size.

Then it will create the ground using the Chrono Vehicle SCMTerrain class. It will choose the size of the ground, and the resolution of the ground mesh, then describe specific ground properties.

Next, we initialize a Spirit object with 0 trusses to a specific location, and enable shoulder and knee joints with given PID gains. Then we add it to our simulation system and enable collisions with the toes and main body with the ground.

Next we create a visualization window, and add lighting and a camera to the scene. Then we begin our main update loop where after a delay, we send commands to the legs to begin a elementary stand procedure. 

### Rover Example
Running the `rover_example` executable will initialize the simulation, handling interrupts  (CTRL-C) cleanly, setting gravity, choosing thread count for the collision system, choosing an integration solver, and finally choosing an integration step size.

Then it will create the ground using the Chrono Vehicle SCMTerrain class. It will choose the size of the ground, and the resolution of the ground mesh, then describe specific ground properties.

Next, we initialize a Rover object to a specific location with 1 truss, and enable wheel joints with given PID gains. Then we add it to our simulation system and enable collisions with the wheels and main body to the ground. Because we did not enable the truss, it will be rigid on the body of the rover.

Next we create a visualization window, and add lighting and a camera to the scene. Then we begin our main update loop where we listen to a ROS2 node to send wheel commands. 


### Variable Stiffness Terrain
Running the `variable_stiffness_example` executable will initialize the simulation, handling interrupts  (CTRL-C) cleanly, setting gravity, choosing thread count for the collision system, choosing an integration solver, and finally choosing an integration step size.

Then it will create the ground using the Chrono Vehicle SCMTerrain class. Then it will create a VariableSoilParams object, which will read the `map.csv` from the `scripts` directory, and initialize a callback to use the given stiffnesses in the `map.csv` for the ground terrain. Afterwards, it will read `map.png` inside `scripts` and color the ground according to its stiffness. NOTE: The `heatmap.py` file in `scripts` must be ran to ensure the map files are up to date.

Next, we initialize a Rover object to a specific location with no trusses, and enable wheel joints with given PID gains. Then we add it to our simulation system and enable collisions with the wheels and main body to the ground.

Next we create a visualization window, and add lighting and a camera to the scene. Then we begin our main update loop where we listen to a ROS2 node to send wheel commands to the rover. 

# Map Creation & Structure
`scipts/heatmap.py` is the main Project Chrono solution for creating stiffness maps inside this package. You can call this script without arguments as `python3 heatmap.py`. This will generate a `.csv` with random stiffnesses throughout the map at a specific grid size. Inside `headmap.py` at the bottom of the script you can designate map size (length and width), resolution (grid size for stiffnesses), stiffness ranges (inside map_object), a "worse resolution factor", and `.csv` and `.png` filename. 

The functionality of this script is as follows:

The program will generate a map with random stiffnesses inside the given size and stiffness range, with a grid size equivalent to the worse resolution factor. Then it will linearly interpolate between these large grids to generate a smooth transition between these larger tiles at the regular grid resolution size that the user set. This new grid is what is used as the output to this map.

## Map CSV Format and Parsing
This resulting map is saved as a `.csv` in the format `x, y, stiffness`. When loaded into Project Chrono, a vorinoi diagram is created with all values inside the given CSV. This vorinoi diagram is used to determine which stiffness to assign to the mesh during each mesh collision during runtime. For visualization, a .png heatmap is generated by the `heatmap.py` script, and the vertically flipped version is applied (due to how images are applied in Chrono) to the mesh to provide an accurate visualization of the stiffness of the ground. 

# Infrastructure Guide
## Sim Tools
Sim tools is the library of classes that we use inside of our execution main scripts. This library is built separately, and needs referenced when attempting to run any of the executables in the `bin`. This is why `ros2_ws/install/setup.bash` needs references, as that is where the built binary for the sim_tools library is stored. When you edit something in sim tools, the whole library needs rebuilt, however the executables may not be rebuilt, and vice-versa for editing just executables. 
### Robot Construction
Our robots in Chrono are not referenced by `URDF` files. Instead they are handbuilt using Chrono functions with as many primitive shapes as possible. To achieve this, we have created robot factories inside sim_tools. These factories are designed to fully build the robots in Chrono at a desired position and orientation, with all the correct motors, and trusses/probes. Then, pass all of the key information to a new robot object which is then given to the user for use. This factory class can create any amount of new robots, and it stores no information from previously created robots, as all of that is passed to the user as the new robot object.
### Robot class
An instance of a robot class should only be created by it's respective robot factory. The class is used to control the robot, and get information from it's various motors and ground interactions. During it's creation, a new object is created- a ROS node for that robot.
### Robot ROS class
For each robot type, a ROS node class exists. This robot_ROS class handles publications, subscriptions, and services related to that robot, and the intercommunication between the Robot class and the Robot ROS class occurs inside the handle_ros function inside the robot class. There is where you can place new data into the ROS class, and pull data from ROS into the robot class itself for uses such as control.

# Development Guide

## Background
First, it is imperative to become at least somewhat familiar with [Project Chrono](https://projectchrono.org), as it is the backbone of what makes this package work. More specifically, we are using their development branch, which can be found as the main branch in their [GitHub repository](https://github.com/projectchrono/chrono). On top of the core Chrono files, we also build four of the given submodules: `Irrlicht`, `Parsers`, `ROS`, and `Vehicle`. Within these four submodules we utilize the following functions. The `Irrlicht` submodule is the visualization submodule of choice, and handles everything related to showing the simulation running on the machine. The `Parsers` submodule allows us to load in URDF files into Chrono, which gives us the ability to not have to use their internal proprietary robot builder. The `ROS` submodule allows us to easily publish things like object pose using ROS handlers custom built by Chrono. It is important to note that this does not prevent or hinder in any way the use of ROS regularly in these files, and this submodule is just a convenience for some of the things that we might be interested in sharing over ROS. The `Vehicle` submodule is what gives us access to the [deformable terrain](https://api.projectchrono.org/7.0.0/vehicle_terrain.html#vehicle_terrain_scm) that we use which is built by referencing the algorithms in [this paper](https://core.ac.uk/download/pdf/77229228.pdf).

## Building a new Simulation
Project Chrono has great [reference documentation](https://api.projectchrono.org/development/) and also an active [google group](https://groups.google.com/g/projectchrono) that is responsive and helpful with any questions pertaining to Project Chrono, especially for projects working on the development branch like this one. On top of that, Project Chrono has really amazing demos that not only show the capabilities of the software, but walk through the code required to build and run them effectively. Referencing those, and the files already in this repository will provide the best idea of how to build a new sim. After creating your `.cpp` file, and populating it with your `main` function and desired functionality, you must also include it as a file to build as an executable inside the `CMakeLists.txt` file located in the `chrono_sim` directory. This should be as simple as referencing the lines that are used to build any of the other executables such as `spirit_example` and mimicking them with the chosen name/location of your file in their place. Afterwards, sourcing ROS (`source /opt/ros/humble/setup.bash`), running
```
colcon build
```
inside your `ros2_ws` directory and sourcing your local workspace
```
source ~/ros2_ws/install/setup.bash
```
should be enough to allow you to run your newly created executable located in the `bin` directory inside the `chrono_sim` package!
## Notes

### Stability
Aside from getting the project to build, and everything placed in simulation as desired, simulation stability seems to be the largest challenge to overcome in producing high-quality simulation data. The [SCM](https://api.projectchrono.org/7.0.0/vehicle_terrain.html#vehicle_terrain_scm) that we use in these experiments was designed with large wheeled / tracked systems that had very large patch contact with the ground, and considerable weight and momentum by themselves. These systems are very unlike most of our mobile robots on this project which have much smaller masses, essentially point contacts on the ground, and considerably less momentum and speed while requiring just as much if not more precision and accuracy of the terrain. 

Instability has arisen thusfar in mostly the contacts between the robots and the ground, but also in the prescribed joints of the robots themselves. These instabilities present in multiple ways, such as quadruped toes bouncing on the terrain, random side-load forces when pushing into the ground, or objects breaking their defined joint restrictions with the resulting overall simulation being minimally to drastically affected. Thankfully there are a few things we can do on the setup side to reduce these issues.

Chrono can run on two types of physics 'systems'- NSC and SMC which represent a non-smooth contact and smooth-contact method respectively. In all the Chrono demos using [SCM](https://api.projectchrono.org/7.0.0/vehicle_terrain.html#vehicle_terrain_scm), they use SMC, and it seems to provide a slightly more stable simulation with minimal performance hit. There are also solver types that Chrono uses to integrate ahead in time, and of those PSOR is the quickest and apparently most inaccurate. However, experimenting with a few of the other solvers with a high max iteration leash such as BARZILAIBORWEIN did not lead to noticablly stabler simulations, while definitely affecting performance.

So with a few of the ways listed that haven't seemed to affect the simulation so far- what has? That answer is mostly through integration step size, a bit in [SCM](https://api.projectchrono.org/7.0.0/vehicle_terrain.html#vehicle_terrain_scm) terrain mesh resolution, and URDF body editing. The integration step size is the amount to step forward in time using the given solver, while taking into account all contacts and external forces like gravity. Reducing this is the single largest factor in increasing the stability of the simulation, but at the same time it is also the single largest performance reducer/gain so a balance must be struck when deciding between speed and accuracy/stability. Because we are using such a unique terrain ([SCM](https://api.projectchrono.org/7.0.0/vehicle_terrain.html#vehicle_terrain_scm)), our ground is defined as a mesh and we can alter this mesh size to increase or decrease the corresponding node count of the ground. More nodes means a better "wrap" around any intruding objects and hopefully better simulation, but also it means higher computational cost. Lastly one of the key factors that has helped the stability of this system is larger momentum on robot bodies. The original Spirit urdf had laughably small weight and momentum values for the toes which are considerably larger in both of those categories than given. These values being almost rounding-error small contributed to much of the "bouncing" behaviour we saw in our simulation tests. Artificially (or maybe "accurately") increasing these values produced a much cleaner and stable simulation result with our any affect on performance! Finally a win for the programmers... There are assuredly more ways that can increase this stability factor, but these are the main ones that have been used thus-far to increase the reliability of the system.

### Chrono [SCM](https://api.projectchrono.org/7.0.0/vehicle_terrain.html#vehicle_terrain_scm) Resources

[Deformable terrain paper](https://core.ac.uk/download/pdf/77229228.pdf)

![SCM Properties](https://github.com/TRUSSES/chrono_sim/assets/15080974/7b9cde53-4c2b-4760-810b-c2971b4d979a)
![SCM Slide 1](https://github.com/TRUSSES/chrono_sim/assets/15080974/0f527035-3715-4c18-94b1-3e11b6b78a48)
![SCM Slide 2](https://github.com/TRUSSES/chrono_sim/assets/15080974/aa7494fc-0909-4ce7-99c3-038011bf8084)
