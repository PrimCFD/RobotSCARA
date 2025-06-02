# 🦾 SCARA Robot Simulation Suite

Welcome to the documentation for the **SCARA Robot Simulation & Control Suite** — a real-time simulation and control platform for SCARA-like parallel robots. This system supports both **Software-in-the-Loop (SIL)** and **Hardware-in-the-Loop (HIL)** workflows.

---

## 🚀 What’s Inside

- ✅ PyQt5 GUI with 3D visualization (via PyVista)
- ✅ Symbolic dynamics using the Equivalent Point Mass (EPM) method
- ✅ MPC and PID-based torque-level inverse dynamics control
- ✅ Adaptive RK4 simulation and real-time fixed-step HIL
- ✅ Cross-platform compatibility (Windows / Linux / macOS)
- ✅ CSV-based trajectory buffering, export, and replay

---

## 📘 Documentation Topics

- [Read Me](./README.md): Project setup and overview
- [Control Architecture](./control.md): Server loop, SIL/HIL logic, and PID/MPC control
- [Kinematics & Dynamics](./kindyn.md): Symbolic model, IK, Jacobians, and dynamic simplification

---

## 🧰 Getting Started

To run the project:

```bash
# Windows
launcher.bat

# Linux/macOS
chmod +x launcher.bash
./launcher.bash
```

For development mode:

```bash
pip install -r requirements.txt
python SCARA.py
```

---

## 📫 Questions or Feedback?

Feel free to open an issue or discussion on the [GitHub repository](https://github.com/yourusername/scara-robot).

---

© 2025 – SCARA Simulation & Control Authors. MIT License.