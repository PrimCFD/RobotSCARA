# Kinematic and Dynamic Model of a Simplified 3-DOF SPARA Robot  
  
## 1. Kinematics  
  
### Structure  
- **Degrees of Freedom**: Pure translational motion (x, y, z).    
- **Actuators**: 3 legs (Legs 1–3), with redundancy and rotational DOFs removed.    
- **Key Changes from Original 4+1 DOF Design**:    
  - Suppressed redundant parameter `γ` (Leg 5 replaced by 3).    
  - Eliminated rotational DOF `φ` (double parallelogram linkages removed).    
  
### Simplified Jacobian Matrices  
Relate joint velocities to Cartesian velocities:    
$$  
\mathbf{J} \dot{\mathbf{c}} = \mathbf{K} \dot{\boldsymbol{\theta}}  
$$  
- **Variables**:    
  - $\dot{\mathbf{c}} = [\dot{x}, \dot{y}, \dot{z}]^T$: End-effector translational velocities.    
  - $\dot{\boldsymbol{\theta}} = [\dot{\theta}_1, \dot{\theta}_2, \dot{\theta}_3]^T$: Joint angular velocities.    
  
### Singularities  
- **Type I (Inverse Kinematics)**: Occurs when legs are fully extended/folded.    
- **Type II (Forward Kinematics)**: Eliminated due to removed rotational constraints.    
  
---  
  
### Inverse Kinematics  
  
### Joint Angle Expressions $( \theta_i )$ 
For legs $i = 1, 2, 3$:    
$$  
\theta_i = 2 \arctan\left( \frac{B_i \pm \sqrt{B_i^2 - (A_i + C_i)(C_i - A_i)}}{A_i + C_i} \right)  
$$    
**Geometric Parameters**:    
For legs $i = 1, 2$:
$$  
\begin{align*}  
A_i &= 2(a_{iy} - b_{iy}) \\  
B_i &= 2(b_{ix} - a_{ix}) \\  
C_i &= \frac{\ell_{i2}^2 - \ell_{i1}^2 - (a_i - b_i)^T (a_i - b_i)}{\ell_{i1}}  
\end{align*}  
$$   
For leg $i = 3$:
$$  
\begin{align*}  
A_i &= 2(a_{iz} - b_{iz}) \\  
B_i &= 2(b_{ix} - a_{ix}) \\  
C_i &= \frac{\ell_{i2}^2 - \ell_{i1}^2 - (a_i - b_i)^T (a_i - b_i)}{\ell_{i1}}  
\end{align*}  
$$  
**Variables**:    
- $a_i$ : Base joint position of leg $i$.    
- $b_i$ : Distal joint position of leg $i$.    
- $\ell_{i1}, \ell_{i2}$: Proximal/distal link lengths.    
  
---  
  
### Workspace  
  
### Translational Workspace  
- **Dimensions**: ~0.5m × 1.0m × 0.3m (x, y, z).    
- **Limits**:    
  - Determined by leg lengths ($\ell_{i1}, \ell_{i2}$).    
  - Constrained by joint ranges (Fig. 10 in the paper).    
  
### Performance  
- **Speed**: Up to 3.5 m/s. 
 
---  
  
### Velocity Equations  
  
### Angular Velocity $\dot{\theta}_i$ 
 
Derived from Jacobian matrices:    
$$  
\dot{\boldsymbol{\theta}} = \mathbf{K}^{-1} \mathbf{J} \dot{\mathbf{c}}  
$$    
  
---  

- Original Paper:    
  **L. -T. Schreiber and C. Gosselin**, "Schönflies Motion PARAllel Robot (SPARA): A Kinematically Redundant Parallel Robot With Unlimited Rotation Capabilities," in *IEEE/ASME Transactions on Mechatronics*, 2019.    
  **DOI**: [10.1109/TMECH.2019.2929646](https://doi.org/10.1109/TMECH.2019.2929646).

## 2. Dynamics

### Abstract

This simulation uses the **Equivalent Point Mass (EPM)** method — a simplified approach for dynamic modeling of parallel robots. The method replaces distal links with dynamically equivalent point masses at their endpoints, eliminating complex angular velocity calculations. Derived from Lagrangian mechanics, this technique is **computationally efficient** while maintaining **good accuracy for slender links**.

---

### Equivalent Point Mass (EPM) Method

#### Core Concept

The EPM method simplifies dynamics by:

- Replacing distal links with point masses at their endpoints (Bᵢ and Cᵢ)  
- Assigning masses based on energy equivalence principles  
- Handling potential and kinetic energy separately  

---

#### Mass Assignment Strategies

**For Potential Energy (exact equivalence):**

$$
m_{p1} = \frac{m l_2}{l_1 + l_2}, \quad 
m_{p2} = \frac{m l_1}{l_1 + l_2}
$$

Where \( l_1, l_2 \) are distances from the center of mass to the endpoints.  
For symmetric links:

$$
m_{p1} = m_{p2} = \frac{m}{2}
$$

---

**For Kinetic Energy (approximate equivalence):**

##### Fixed Mass Method:
- \( m_{k1} = m_{k2} = \frac{m}{2} \) (simple but less accurate)  
- or \( m_{k1} = \frac{m}{3}, \quad m_{k2} = \frac{2m}{3} \) (better for rotational inertia)

##### Variable Mass Method (higher accuracy):

$$
\begin{align*}
m_{k1} &= \frac{m}{3}(1 + k_1) \\
m_{k2} &= \frac{m}{3}(1 + k_2)
\end{align*}
$$

With \( k \)-coefficients calculated via minimum-norm solution:

$$
k_1 = \frac{|\mathbf{v}_1|^2}{|\mathbf{v}_1|^4 + |\mathbf{v}_2|^4} \mathbf{v}_1^T \mathbf{v}_2, \quad
k_2 = \frac{|\mathbf{v}_2|^2}{|\mathbf{v}_1|^4 + |\mathbf{v}_2|^4} \mathbf{v}_1^T \mathbf{v}_2
$$

---

### Dynamic Model Implementation

#### Energy Formulation

Kinetic energy for distal links simplifies to:

$$
T_{\text{EPM}} = \frac{1}{2} (m_{k1} \mathbf{v}_1^T \mathbf{v}_1 + m_{k2} \mathbf{v}_2^T \mathbf{v}_2)
$$

---

#### Actuator Torques

Total torque combines contributions from:

**Proximal links (joint space):**

$$
\boldsymbol{\tau}_a = \frac{d}{dt}\left( \frac{\partial T}{\partial \dot{\boldsymbol{\theta}}} \right) - \frac{\partial T}{\partial \boldsymbol{\theta}} + \frac{\partial V}{\partial \boldsymbol{\theta}}
$$

**Platform + distal points (Cartesian space):**

$$
\mathbf{f} = \frac{d}{dt}\left( \frac{\partial T}{\partial \dot{\mathbf{p}}} \right) - \frac{\partial T}{\partial \mathbf{p}} + \frac{\partial V}{\partial \mathbf{p}}
$$

**Total actuator torque:**

$$
\boldsymbol{\tau} = \boldsymbol{\tau}_a + (\mathbf{J}^{-1}\mathbf{K})^T \mathbf{f}
$$

---

### Advantages & Limitations

#### ✔️ Advantages

- Eliminates complex angular velocity calculations  
- Reduces computational cost by 30–60% vs exact models  
- Maintains >95% accuracy for slender links  
- Fixed mass method enables real-time control  
- Variable mass improves accuracy for known trajectories  

#### ❌ Limitations

- Accuracy decreases for thick/rotating links  
- Fixed masses: limited accuracy across configurations  
- Variable masses: requires known trajectories  
- Potential energy: exact only when COM lies on link axis  

---

### Performance Comparison

| Method             | Comp. Cost | Torque RMSE | Best For             |
|--------------------|------------|-------------|----------------------|
| Fixed Mass EPM     | Lowest     | 5–6 Nmm     | Real-time control    |
| Variable Mass      | Medium     | 2–3 Nmm     | Known trajectories   |
| Slender Link*      | High       | 0.4 Nmm     | General-purpose      |

> \* Reference method from original paper

---

### Conclusion

The EPM method provides an effective balance between **accuracy** and **computational efficiency**. For **real-time control**, fixed masses (\( m/2 \) or \( m/3 + 2m/3 \)) are recommended. For **trajectory-based tasks**, variable masses offer superior accuracy with moderate computation overhead.

---

- Original Paper: 
  Zhou Z., Gosselin C. (2024) *Simplified Inverse Dynamic Models of Parallel Robots Based on Lagrangian Approach*.  
  **Meccanica**, 59:657–680. DOI: [10.1007/s11012-024-01782-6](https://doi.org/10.1007/s11012-024-01782-6)
