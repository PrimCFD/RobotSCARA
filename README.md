# 🦾 SCARA Robot Simulation & Control GUI
![SCARA Robot Visualization](./assets/images/scara_icon.svg)  
*Advanced simulation and control platform for SCARA-type robots*

[![Python 3.8+](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version 0.90](https://img.shields.io/badge/version-0.90-blue)](https://github.com/yourusername/scara-robot/releases)

## 🌟 Features
- **3D Visualization** - PyVista-based real-time kinematic visualization
- **Dynamic Modeling** - Joint torque calculations and motor dynamics
- **Workspace Analysis** - Reachability analysis and collision detection
- **Trajectory Planning** - Angular velocity/acceleration profiling
- **Data Logging** - CSV export/import for trajectory analysis
- **Cross-platform** - Windows/Linux/macOS support

## Setup


#### Launcher :

![Launcher](/assets/images/Launcher.gif)

### Automatic Setup (Recommended)
**Windows**:


>`launcher.bat`
   
**Linux/macOS**:


>`chmod +x launcher.sh`
>
>`./launcher.sh`

The launcher will:

- Verify Python 3.8+ installation
- Install required dependencies automatically
- Launch the application

**Manual Setup**

*For advanced users*:

>`pip install -r requirements.txt`
>
## Running the program

**Standard Launch**

Use the launcher script for best experience:

**Windows**: Double-click launcher.bat

**Linux/macOS**: Execute ./launcher.sh

**Developer Mode**

For direct execution:

>`python SCARA.py`

## GUI

The features include :
> 1. [x] Visualizing current geometry with the PyVista Render
> 2. [x] Choosing target angles with 0.1 deg precision
> 3. [x] Animating the trajectory
> 4. [x] Computing joint torques for trajectory control
> 5. [x] Computing the Workspace accurately
> 6. [x] UX friendly GUI
> 7. [x] Exporting data as a csv file for analysis

Upcoming features :

> 2. [ ] Running a simulink PID stable controller
> 3. [ ] Starting/Stopping data acquisition
> 4. [ ] Having a real-time plot of measured vs shown trajectory

---
#### Elbow position change:

![Elbow](/assets/images/Elbow.gif)

#### Target position change :

![Target](/assets/images/Target.gif)

#### Plot of motors kinemetics/dynamics :

![Plot](/assets/images/Plot.gif)

#### Plot of Workspace / Spherical workspace :

![Workspace](/assets/images/Workspace.gif)

#### Measure and Buffering :

![Buffer](/assets/images/Buffer.gif)

#### Save trajectories :

![Save](/assets/images/Save.gif)

#### Replay saved trajectories :

![Replay](/assets/images/Replay.gif)

## Key Functionalities

### Robot Control

- Real-time joint angle adjustment (0.1° precision)
- Elbow position configuration
- Velocity/acceleration constraints setup

### Visualization Tools

- 3D workspace rendering
- Trajectory animation
- Dynamic solution pathways (A/B configurations)

### Analysis Features

- Motor torque calculations
- Velocity/acceleration profiling
- Inverse dynamics computations

### Data Management

- Trajectory logging/replay
- CSV export/import
- Experiment metadata management

## Project structure

> - **./assets/**
>   - *scara_icon.ico* &rarr; Windows Desktop icon
>   - *scara_icon.png* &rarr; Linux/MacOS Desktop icon
>   - **./images/**
>     - *plot_icon.svg*
>     - *scara_icon.svg*
>     - *{Title}.gif* &rarr; README gifs
> - **./backend/**
>    - **./build/** &rarr; CMake build files
>    - **./include/**
>      - **./Eigen/** &rarr; Linear algebra library
>      - *Controller.hpp* &rarr; Controller class header
>      - *HardcodedParams.hpp* &rarr; Generated header file hardcoding parameters
>      - *RobotModel.hpp* &rarr; RobotModel class header
>      - *SIL.hpp* &rarr; Header to be in server scope
>    - **./src/**
>     - *Controller.cpp* &rarr; Controller
>     - *main_server.cpp* &rarr; IPC Server
>     - *HIL.cpp* &rarr; Hardware in the loop
>     - *RobotModel.cpp* &rarr; Dynamics computations
>     - *SIL.cpp* &rarr; Software in the loop simulation
>    - *CMakeLists.txt* &rarr; Builds backend for SIL or HIL
> - **./bin/** &rarr; C++ binary : Server  + SIL/HIL
> - **./buffer/**
>   - *buffer.csv* &rarr; Trajectory buffered before save (t, θ, φ)
> - **./configs/**
>   - *initial.json* &rarr; Geometrical configuration on launch 
>   - *temp.json* &rarr; Current Geometrical configuration 
> - **./data/** &rarr; Trajectory saving directory
>   - **/Experiment_1/** &rarr; Experiment directory
>     - *metadata.json* &rarr; Config file loaded when Experiment directory created
>     - *Trajectory_1.csv* &rarr; Trajectories (t, θ, φ)
>     - *Trajectory_2.csv*
>     - ...
>   - **/Experiment_2/**
>     - *metadata.json*
>     - *Trajectory_1.csv*
>     - *Trajectory_2.csv*
>     - ...
> - **./docs/**
>   - [*control.md*](./docs/control.md) / [*kindyn.html*](./docs/kindyn.html) &rarr; Theory / choices for robot control
>   - [*kindyn.md*](./docs/kindyn.md) / [*kindyn.html*](./docs/kindyn.html) &rarr; Theory / choices for kinematic/dynamic modeling
> - **./robot/**
>   - *\_\_init__.py*
>   - *analytic.py* &rarr; Dynamic modeling
>   - *gui.py* &rarr; **PyQt** Main window 
>   - *hil_client.py* &rarr; IPC Client for HIL
>   - *kinematics.py* &rarr; Inverse Kinematics (IK)
>   - *misc.py* &rarr; Miscallenous functions
>   - *scene.py* &rarr; **PyVista** QtInteractor widget
>   - *sil_client.py* &rarr; IPC Client for SIL
>   - *sil_process.py* &rarr; SIL Server processing
> - **./tests/**
>   - **./integration/** &rarr; Checks **PyQt** creation
>     - *integration_test.py*
>   - **./performance/** &rarr; Checks performance
>     - *dynamics_perf_test.py*
>     - *gui_perf_test.py*
>     - *kinematics_perf_test.py*
>     - *memory_perf_test.py*
>   - **./unit/** &rarr; Checks main functions
>     - *config_test.py*
>     - *conftest.py*
>     - *dynamics_test.py*
>     - *kinematics_test.py*
>     - *velocity_profile_test.py*
> - **./utils/** &rarr; Python utility files
> - - *generate_params.py* &rarr; Generates *HardcodedParams.hpp* at compile time
>   - *jacobians.py* &rarr; Sympy computation of Joint/Task spaces jacobians
>   - *jacobians_dot.py* &rarr; Sympy computation of Joint/Task spaces jacobians derivatives
>   - *litteral.py* &rarr; EPM method EOM computation : M, G matrices
> - .*gitignore*
> - *launcher.bash* &rarr; Windows Launcher executable
> - *launcher.bat* &rarr; Linux/MacOS Launcher Executable
> - *launcher.py* &rarr; Launcher script
> - *LICENSE*
> - *README.md / README.html*
> - *requirements.txt*
> - *SCARA.py* &rarr; Application Entry point

## Control

The simulink *C++* export binary is located inside ***./bin/***, for additional details you can read more in [***./docs/control.md***](./docs/control.md) 