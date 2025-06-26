# 🦾 SCARA Robot Simulation & Control Suite

![SCARA Robot Visualization](./assets/images/scara_icon.svg)  
**Advanced real-time simulation and control platform for 3-DOF SCARA-like parallel robots**

[![Python 3.8+](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version 0.90](https://img.shields.io/badge/version-0.90-blue)](https://github.com/yourusername/scara-robot/releases)

---

## 🌟 Key Features

- ✅ **3D Real-Time Visualization** (PyVista + PyQt5)
- ✅ **Analytical Inverse Kinematics & Workspace Analysis**
- ✅ **Symbolic Dynamics via EPM (Equivalent Point Mass)**
- ✅ **SIL/HIL Dual Simulation Architecture**
- ✅ **Inverse Dynamics and Torque MPC Controller**
- ✅ **Trajectory Profiling + Velocity/Acceleration Control**
- ✅ **Cross-platform: Windows / Linux / macOS**
- ✅ **Data Logging, Buffering, and Trajectory Replay**

---

## 🚀 Quick Start

### 🔁 One-Line Setup

#### 🪟 Windows:
```bash
launcher.bat
```

#### 🐧 Linux/macOS:
```bash
chmod +x launcher.bash
./launcher.bash
```

This will:
- Set up Python virtual environment
- Install dependencies
- Build the C++ backend
- Launch the GUI

![Launcher Demo](assets/images/Launcher.gif)

---

## 💻 GUI Features


| Feature                     | Description                                                  |
|----------------------------|--------------------------------------------------------------|
| 🛰 3D Scene Viewer          | Real-time render using PyVista and QtInteractor              |
| 🎯 Target Setter            | Precision pose input (±0.1°/mm)                              |
| 🎞 Trajectory Animation     | Time-interpolated motion preview                            |
| ⚙️ Torque Solver            | MPC + inverse dynamics with full robot model                |
| 📊 Plotting Panel           | Velocity, acceleration, and torque visualization            |
| 🧠 Workspace Mapper         | Computes reachable volume and displays spherical workspace  |
| 💾 Data Logging             | CSV export + replay for recorded trajectories               |

---

## 🧠 Architecture Overview

The project uses a **modular client-server design** for simulation and control:

```mermaid
%% SCARA robot – expanded data-flow / protocol map
%%{init: { "flowchart": {
      "layout": "elk",
      "ranker": "tight-tree",
      "rankSpacing": 50,
      "nodeSpacing": 10
} } }%%
graph TD
  %% ───────── Desktop (Python) ─────────
  subgraph Desktop_Python["Desktop (Python)"]
    SILC["client.py"]
    SensorTh["serial_sensor.py<br/>thread"]
        GUI["GUI<br/>PyQt + PyVista"]


  end

  %% ───────── C++ back-end binaries ─────────
  subgraph CPP_Server["C++ server (backend/bin)"]
    CppSrv["main_server.cpp<br/>TCP bridge + dispatcher"]
    SIL["SIL.cpp<br/>RK4_5 at 1 kHz<br/>EPM dynamics + LQR MPC"]
    HIL["HIL.cpp<br/>1000 Hz loop<br/>6-state Kalman + MPC"]
    CppSrv -->|mode S| SIL
    CppSrv -->|mode H| HIL
  end

  %% ───────── TCP bridge ─────────
  subgraph TCP_Bridge["TCP localhost 5555"]
    SILC -->|handshake header 1B mode 3 len strings| CppSrv
    SILC -->|trajectory block int32 n elbow xyz l_arm 80B n| CppSrv
    CppSrv -->|128B frame t x3 xdot3 theta3 thetadot3 tau3 1kHz 1000Hz| SILC
  end

  %% SIL feedback to server
    SIL -->|mirror 128B frame| CppSrv

  %% ───────── Hardware ─────────
  subgraph Realtime_HW["HW"]
    Sensor["Spherical angles sensors <br/>Potentiometers 0.3 deg<br/>t r theta phi CSV"]
    Arduino["Arduino UNO R3<br/>250 kbaud<br/>freq to STEP DIR"]
    EM["EM556s drivers x3<br/>200 kHz max"]
    Motor["NEMA34 steppers"]

    HIL -->|read CSV line| Sensor
    HIL -->|ASCII cmd F fx fy fz dirBits| Arduino
    Arduino -->|STEP DIR TTL| EM
    EM --> Motor

  end

  SensorTh-->|read CSV line|Sensor


  %% ───────── Styling classes ─────────
  classDef py  fill:#e8f7ff,stroke:#0d8bf2,color:#036
  classDef cpp fill:#fffbe6,stroke:#ec9c00,color:#8a5b00
  classDef hw  fill:#f4faff,stroke:#1f6e43,color:#104623
  class GUI,SensorTh,SILC py
  class CppSrv,SIL,HIL cpp
  class Sensor,Arduino,EM,Motor hw

```

> ✅ Supports both **Software-in-the-Loop (SIL)** and **Hardware-in-the-Loop (HIL)** execution.

---

## 🧮 Core Algorithms

### ⚙️ Dynamics

The system uses a **Lagrangian model with Equivalent Point Mass (EPM)** for distal links, enabling:

- ✔️ Simplified mass distribution without angular velocity terms
- ✔️ Real-time computation of task-to-joint dynamics
- ✔️ Feedforward inverse dynamics via:
  $$
  \tau = M  \ddot{\theta} + G
  $$

See [`docs/kindyn.md`](./docs/kindyn.md) for theory.

---

### 🧾 Control Pipeline

- ✅ **Quintic trajectory interpolation** (C² smoothness)
- ✅ **PID + Feedforward torque control**
- ✅ **MPC-based acceleration inverse dynamics**
- ✅ **Adaptive RK4 with Bogacki-Shampine integration (SIL)**
- ✅ **Real-time 1kHz controller loop (HIL)**

For details, refer to [`docs/control.md`](./docs/control.md)

---

## 📂 Project Layout

<details>
<summary>Click to expand file tree</summary>

```bash
📦 root/
├── assets/               # Icons & media
│   ├── scara_icon.ico/png
│   └── images/           # GIFs, SVGs
├── backend/              # C++ core (dynamics, control)
│   ├── build/            # CMake output
│   ├── include/          # C++ headers (RobotModel, Controller, etc.)
│   ├── src/              # C++ source (SIL.cpp, HIL.cpp, etc.)
│   └── CMakeLists.txt
├── bin/                  # Compiled binaries
├── buffer/               # Trajectory buffer (temp)
│   └── buffer.csv
├── configs/              # Initial and runtime configurations
├── data/                 # Experiment outputs
│   └── Experiment_X/
│       ├── metadata.json
│       └── Trajectory_1.csv
├── docs/                 # Theory and design notes
│   ├── control.md
│   └── kindyn.md
├── robot/                # Python GUI & logic
├── tests/                # Unit, integration, performance tests
├── utils/                # Code generation, symbolic derivation
│   └── generate_params.py
├── SCARA.py              # Python entry point
├── launcher.bat/.bash/.py
├── mkdocs.yml
├── requirements.txt
└── README.md
```
</details>

---

## 📈 Performance Benchmarks

| Mode     | Timing      | Real-Time Capable | Notes                       |
|----------|-------------|-------------------|-----------------------------|
| SIL      | ~200–500μs  | ❌ No              | Uses adaptive RK4 integration |
| HIL      | ~950μs      | ✅ Yes             | Fixed-step 1kHz loop          |

---

## 🧪 Testing

Run unit + integration tests with:
```bash
pytest tests/
```

Includes:
- 🧠 `robot/kinematics.py`: IK + FK coverage
- 🚀 `backend/RobotModel.cpp`: Dynamics validation
- 📊 `performance/`: GUI + dynamics timing

---

## 📚 References

- Zhou Z., Gosselin C. (2024)  
  *Simplified Inverse Dynamic Models of Parallel Robots Based on Lagrangian Approach*,  
  **Meccanica**, 59:657–680  
  [DOI: 10.1007/s11012-024-01782-6](https://doi.org/10.1007/s11012-024-01782-6)

- Schreiber & Gosselin (2019)  
  *Schönflies Motion PARAllel Robot (SPARA)*, IEEE/ASME Trans. on Mechatronics  
  [DOI: 10.1109/TMECH.2019.2929646](https://doi.org/10.1109/TMECH.2019.2929646)

---

## ✅ Status

| Module       | State         | Notes                                 |
|--------------|---------------|---------------------------------------|
| GUI          | ✅ Stable      | PyQt5 + PyVista                      |
| SIL          | ✅ Operational | Adaptive dynamics simulation          |
| HIL          |  To be tested | Real-time control with fixed-rate     |
| IK/FK        | ✅ Validated   | Analytical/Iterative with error handling  |
| Dynamics     | ✅ Verified    | Lagrangian EPM model                  |
| Logging      | ✅ Complete    | Replay and export ready               |


---

## 📄 License

This project is licensed under the MIT License – see the [LICENSE](./LICENSE) file for details.