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
This simulation uses the **slender link method** - a simplified approach for dynamic modeling of parallel robots. The method enables efficient computation of distal link kinetic energy using endpoint velocities rather than complex angular velocity expressions. Derived from Lagrangian mechanics, this technique is particularly effective for robots with slender distal links undergoing high-speed motion.

---

### Slender Link Method

### Key Concept
For distal links in parallel robots:
- Traditional kinetic energy calculation requires angular velocity and center-of-mass velocity
- Slender link method approximates energy using **endpoint velocities only**
- Assumes:
  - Uniform mass distribution along link
  - Ideal slenderness (negligible cross-sectional dimensions)

### Velocity Relationship
For a distal link between points $B_i$ and $C_i$:
$$\mathbf{v}_2 - \mathbf{v}_1 = \boldsymbol{\omega} \times \mathbf{d}$$

where:

-   $\mathbf{v}_1, \mathbf{v}_2$​  = Endpoint velocities
    
-   $\boldsymbol{\omega}$= Angular velocity
    
-   $\mathbf{d}$ = Link vector (from  BiBi​  to  CiCi​)
    

### Kinetic Energy Formulation

The kinetic energy of a slender link is calculated as:

$$
T_s = \frac{1}{2} \left( \frac{m}{3}\mathbf{v}_1^T\mathbf{v}_1 + \frac{m}{3}\mathbf{v}_2^T\mathbf{v}_2 + \frac{m}{3}\mathbf{v}_1^T\mathbf{v}_2 \right)
$$

**Derivation**:

1.  Express velocity at any point  xx  along the link:
$$
\mathbf{v}_x = \mathbf{v}_1 + \frac{x}{l}(\mathbf{v}_2 - \mathbf{v}_1)
$$

2.  Integrate kinetic energy over link length  ll:
$$
T_s = \int_0^l \frac{1}{2} \frac{m}{l} \mathbf{v}_x^T\mathbf{v}_x dx
$$
----------

### Dynamic Model Implementation

### Jacobian Relationships

Endpoint velocities are related through robot kinematics:

$$
\mathbf{J}\dot{\mathbf{p}} = \mathbf{K}\dot{\boldsymbol{\theta}}
$$

where:

-   $\mathbf{J}, \mathbf{K}$  = Jacobian matrices
    
-   $\dot{\mathbf{p}}$​  = Cartesian velocities
    
-   $\dot{\boldsymbol{\theta}}$ = Joint velocities
    

### Actuator Torques

Total torque combines contributions from:

1.  Links (Joint space : proximal and distal)
    $$
\boldsymbol{\tau}_a = \frac{d}{dt}\left( \frac{\partial T}{\partial \dot{\boldsymbol{\theta}}} \right) 
-\frac{\partial T}{\partial \boldsymbol{\theta}} 
+\frac{\partial V}{\partial \boldsymbol{\theta}}
$$
2.  Platform (Cartesian space)
    $$
\mathbf{f} = \frac{d}{dt}\left( \frac{\partial T}{\partial \dot{\mathbf{p}}} \right) 
-\frac{\partial T}{\partial \mathbf{p}} 
+\frac{\partial V}{\partial \mathbf{p}}$$
3.  Total torque delivered by actuators
$$
\boldsymbol{\tau} = \boldsymbol{\tau}_a + (\mathbf{J}^{-1}\mathbf{K})^T \mathbf{f}$$

----------

### Advantages & Limitations

### ✔️ Advantages

-   Eliminates need for complex angular velocity calculations
    
-   Reduces computational complexity by 40-60% vs exact models
    
-   Maintains good accuracy for slender links (typical RMSE <5%)
    
-   Enables real-time control implementation
    

### ❌ Limitations

-   Accuracy decreases for thick/rotating links
    
-   Requires endpoint velocity computation
    
-   Assumes ideal mass distribution
    


----------

### Conclusion

The slender link method provides an effective balance between accuracy and computational efficiency for parallel robots with slender distal links. While not universally applicable, it enables practical implementation of model-based control strategies in high-speed applications.

---  
- Original Paper:    
**Zhou Z., Gosselin C.**  (2024)  _Simplified Inverse Dynamic Models of Parallel Robots Based on Lagrangian Approach_. Meccanica 59:657-680. DOI:[10.1007/s11012-024-01782-6](https://doi.org/10.1007/s11012-024-01782-6)