# Binary Protocol Specification

> Defines the byte‑level messages exchanged between the **Python client / GUI** and the unified **C++ server** (`main_server.cpp`) over TCP port **5555**. All multi‑byte scalars are *little‑endian*.

---

## 0. Message Flow

```
Client ── Handshake ──▶ Server
Client ── Trajectory ─▶ Server
Server ── Frames ─────▶ Client   (SIL & HIL)
Server ── Ideal pkt ──▶ Client   (SIL‑only)
```

---

## 1. Handshake (Client → Server)

| Offset | Type   | Field         | Description                                                            | Example        |
| ------ | ------ | ------------- | ---------------------------------------------------------------------- | -------------- |
| 0      | `u8`   | `mode`        | ASCII `'S'` (Software‑in‑the‑Loop) **or** `'H'` (Hardware‑in‑the‑Loop) | `'H'`          |
| 1      | `u8`   | `lenSensor`   | Length of `sensor_dev` string (bytes) *(HIL only)*                     | `9`            |
| 2‑…    | `char` | `sensor_dev`  | Serial device for angle sensor                                         | `/dev/ttyUSB0` |
| ?      | `u8`   | `lenArduino`  | Length of `arduino_dev` string *(HIL only)*                            | `4`            |
| ?      | `char` | `arduino_dev` | Serial device for Arduino step‑driver bridge                           | `auto`         |

In **SIL** mode the handshake stops after the single `mode` byte.

---

## 2. Trajectory Payload (Client → Server)

Immediately after the handshake, the client sends a *legacy* fixed‑layout block used by both SIL & HIL paths.

| Offset | Type     | Field   | Count | Notes                                       |
| ------ | -------- | ------- | ----- | ------------------------------------------- |
| 0      | `int32`  | `nWp`   | 1     | Number of way‑points (≤ 1 000 000)          |
| 4      | `double` | `elbow` | 3     | Shoulder‑centred elbow position **(x,y,z)** |
| 28     | `double` | `l_arm` | 1     | Prosthetic arm length                       |
| 36     | `struct` | `wp[i]` | `nWp` | Repeated *way‑point* block (see below)      |

**Way‑point block** (`<10 × double = 80 B>`):

```
{ t, x[3], x_dot[3], x_ddot[3] }
```

`t` is absolute time [s] from trajectory start.

---

## 3. Streaming Frame (Server → Client, SIL & HIL)

During execution the server streams **128‑byte frames** at either the adaptive SIL step or the fixed HIL loop rate.

| Offset | Type     | Field       | Count |                    |
| ------ | -------- | ----------- | ----- | ------------------ |
| 0      | `double` | `t`         | 1     |                    |
| 8      | `double` | `x`         | 3     | Cartesian position |
| 32     | `double` | `x_dot`     | 3     | Cartesian velocity |
| 56     | `double` | `theta`     | 3     | Joint angles (rad) |
| 80     | `double` | `theta_dot` | 3     | Joint velocities   |
| 104    | `double` | `tau`       | 3     | Joint torques (Nm) |

Total: **16 doubles = 128 bytes**.

---

## 4. Ideal‑Trajectory Packet (Server → Client, **SIL‑only**)

Once the numerical simulation finishes, SIL sends an *ideal* torque profile computed with the high‑fidelity model. The packet begins with a 12‑byte header after a SIL empty frame padding followed by `k` identical blocks.

| Offset | Type     | Field      | Count | Notes                                       |
| ------ | -------- | ---------- | ----- | ------------------------------------------- |
| 0      | `double` | `endTime`  | 1     | Final simulation time (s)                   |
| 8      | `int32`  | `k`        | 1     | Number of way‑points in original trajectory |
| 12     | `struct` | `ideal[i]` | `k`   | Repeated *ideal* block                      |

**Ideal block** (`<10 × double = 80 B>`):

```
{ t, theta[3], theta_dot[3], tau_ideal[3] }
```

---

## 5. Serial Device Auto‑Detection (HIL convenience)

If the `sensor_dev`/`arduino_dev` strings in the handshake are empty, defaults are used:

- `sensor_dev` → `/dev/ttyUSB0`
- `arduino_dev` → `'auto'` → server tries to locate a compatible Arduino UNO by **VID/PID** or `/dev/serial/by-id` symlink, mirroring the client‑side logic in `serial_sensor.py`. 

---

## 6. Limits & Constants

| Symbol                  | Value         | Defined in                                            |
| ----------------------- | ------------- | ----------------------------------------------------- |
| `MAX_TRAJECTORY_POINTS` | 1 000 000     | `main_server.cpp`              |
| Streaming frame size    | 128 B         | `Frame` struct in both SIL & HIL                      |
| Ring buffer length      | 2 000 samples | `HIL.cpp` TrajectoryRingBuffer |


