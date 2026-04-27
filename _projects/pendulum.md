---
layout: default
title: Inverted Pendulum Verification
---

# Inverted Pendulum Verification
___
___
## 1. Overview and Credits

Conducted reachability analysis of an inverted double pendulum controller, verifying a safe set of initial parameters.

**Project Members** \
Andrew Shim (akshim2@illinois.edu) \
Richard Engel (reengle2@illinois.edu)

This project was completed for ECE 584 at UIUC. The class was taught in Fall 2025 by Professor Mitra (mitras@illinois.edu)


**Tech Stack** 
* Languages: C++
* Tools: [Flow*](https://github.com/chenxin415/flowstar]) 
* Key Concepts: verification of control systems, non-linear hybrid systems, taylor model flowpipes

**Related Links** \
[Report](https://github.com/andykshim/Inv-Pendulum-Verification/blob/main/InvPend_Final_Report.pdf) \
[Github](https://github.com/andykshim/Inv-Pendulum-Verification)

---

## 2. Problem Definition

Cyberphysical systems have become a highly relevant part of everyone's lives. Some applications, such as autonomous driving, robotics, and defense, are high-profile, invoking widespread debates about safety and ethical concerns. Other applications have silently integrated themselves as an irreplaceable part of daily life: traffic management, energy gird monitoring, medical equipements, manufactuing, home appliances, etc. These controllers and algorithms dictate many invisible decisions that have real-world implications, from people's safety and well-being to economic productivity. As such, the verification of cyberphysical systems' controllers and algorithms is a crucial field of research today.

The double inverted pendulum is a well-known controller design problem due to its chaotic, non-linear behavior. To design and verify controllers for these kinds of systems is a non-trivial task. The double-pendulum system is often used as a benchmark for verification tools, testing for success/failure of verification as well as the time taken (cite).

This project aims to utilize one such tool, Flow*, in order to (1) perform a successful verification of a double inverted pendulum system, and (2) use it to explore a safe set of initial parameters with varying angles and angular velocities.

---

## 3. Model Derivation and Simulation Implementation

<div style="text-align: center;">                                                                                                                                                                                 
<img src="/assets/pendulum/DIPC.png" alt="DIPC system diagram" width="390">                                                                                                                                       
</div>  


**Lagrangian Derivation and Linearization**

The Double Inverted Pendulum Controller (DIPC) system consists of six variables, shown here as $(\theta_0, \theta_1, \theta_2, \dot{\theta_0}, \dot{\theta_1}, \dot{\theta_2})$. $\theta_0$ and $\dot{\theta_0}$ are the position and velocity of the cart, $\theta_1$ and $\dot{\theta_1}$ are the angle from vertical and angular velocity of the lower arm of the double pendulum, and $\theta_2$ and $\dot{\theta_2}$ are the angle from vertical and angular velocity of the upper arm of the double pendulum. 
We define $m_0 = 1.5$ for the mass of the cart, $m_1 = m_2 = 0.5$ for the masses of the lower and upper arm, $L_1 = L_2 = 0.5$ for the lengths of the lower and upper arms, and $g = 9.8$ for gravitational acceleration.

The equations of motion for this system can be written in a matrix form:

$$ \textbf{D}(\theta)\ddot{\theta}+\textbf{C}(\theta,\dot{\theta})\dot{\theta}+\textbf{G}(\theta)=\textbf{H}u $$

where

$$ \textbf{D}(\theta)=\begin{pmatrix}
            d_1 & d_2\cos(\theta_1) & d_3\cos(\theta_2) \\
            d_2\cos(\theta_1) & d_4 & d_5\cos(\theta_1-\theta_2) \\
            d_3\cos(\theta_2) & d_5\cos(\theta_1-\theta_2) & d_6
        \end{pmatrix} $$

$$ \textbf{C}(\theta,\dot{\theta})=\begin{pmatrix}
            0 & -d_2\sin(\theta_1)\dot{\theta_1} & -d_3\sin(\theta_2)\dot{\theta_2} \\
            0 & 0 & d_5\sin(\theta_1-\theta_2)\dot{\theta_2} \\
            0 & -d_5\sin(\theta_1-\theta_2)\dot{\theta_1} & 0
        \end{pmatrix} $$

$$ \textbf{G}(\theta)=\begin{pmatrix}
            0 \\
            -f_1\sin(\theta_1) \\
            -f_2\sin(\theta_2)
        \end{pmatrix} $$

$$ \textbf{H}=\begin{pmatrix}
            1 & 0 & 0
        \end{pmatrix}^T $$

The coefficients in these matrices are defined as:

$ d_1 = m_0 + m_1 + m_2 $

$ d_2 = (\frac{1}{2}m_1 + m_2)L_1 $

$ d_3 = \frac{1}{2}m_2L_2 $

$ d_4 = (\frac{1}{3}m_1 + m_2)L_1^2 $

$ d_5 = \frac{1}{2}m_2L_1L_2 $

$ d_6 = \frac{1}{3}m_2L_2^2 $

$ f_1 = (\frac{1}{2}m_1 + m_2)L_1g $

$ f_2 = \frac{1}{2}m_2L_2g $

Finally,  we can derive the state space dynamic equations:

$$ \dot{\textbf{x}} = \begin{pmatrix}
        \textbf{0} & \textbf{I} \\
        \textbf{0} & -\textbf{D}^{-1}\textbf{C}
    \end{pmatrix}\textbf{x} + \begin{pmatrix}
        \textbf{0} \\
        -\textbf{D}^{-1}\textbf{G}
    \end{pmatrix} + \begin{pmatrix}
        \textbf{0} \\
        \textbf{D}^{-1}\textbf{H}
    \end{pmatrix}u  $$


This system also includes an input variable $u$, which is the force applied horizontally to the cart by the controller.

**Linear Quadratic Regulator**

The LQR controller aims to minimize the following cost function:

$$ J_t=\sum_{k=t}^{t_{final}}L_k(\textbf{x}_k, \textbf{u}_k) $$

This represents the total accumulated cost for every state $ \textbf{x}\_k $ and control $ \textbf{u}\_k $ 
from time $ t $ to $ t_{final} $. $ L_k $ corresponds to Linear Qadratic cost:

$$ L_k(\textbf{x}_k,\textbf{u}_k) = \textbf{x}_k^T\textbf{Qx}_k+\textbf{u}_k^T\textbf{Ru}_k $$ 

Here, the $ \textbf{Q} $ matrix represents the cost of deviation from the desired state, and the $ \textbf{R} $ matrix represents the cost of controller input. Both are user-defined. The dynamics equations for SIPC and DIPC can then be linearized around $ \textbf{x}=\textbf{0} $ as such:
$$ \dot{\textbf{x}}=\textbf{Ax}+\textbf{B}u $$

To implement digital control, matrices A and B are approximately discretized into $\boldsymbol\Phi\approx e^{\textbf{A}\Delta t}$ and $\boldsymbol\Gamma\approx\textbf{B}\Delta t$. Optimal digital LQR control is then given by:

$$    u_k = -\textbf{R}^{-1}\boldsymbol\Gamma^T\textbf{Px}_k\equiv-\textbf{Kx}_k $$


We define $ \textbf{Q} $ and $\textbf{R}$ as such:

$$
\begin{aligned}
    \textbf{Q} &= \begin{pmatrix}
            5 & 0 & 0 & 0 & 0 & 0 \\
            0 & 50 & 0 & 0 & 0 & 0 \\
            0 & 0 & 50 & 0 & 0 & 0 \\
            0 & 0 & 0 & 20 & 0 & 0 \\
            0 & 0 & 0 & 0 & 700 & 0 \\
            0 & 0 & 0 & 0 & 0 & 700
        \end{pmatrix} \\
    
    \textbf{R} &= 0.5
\end{aligned}
$$

Finally, the linearized $\textbf{A}$, $\textbf{B}$, and the derived $\textbf{K}$ matrices:

$$    
        \textbf{A} = \begin{pmatrix}
            0 & 0 & 0 & 1 & 0 & 0 \\
            0 & 0 & 0 & 0 & 1 & 0 \\
            0 & 0 & 0 & 0 & 0 & 1 \\
            0 & -5.75217 & 0.63913 & 0 & 0 & 0 \\
            0 & 63.1913 & -26.8435 & 0 & 0 & 0 \\
            0 & -80.5304 & 67.7478 & 0 & 0 & 0
        \end{pmatrix}
        $$

$$
        \textbf{B} = \begin{pmatrix}
            0 \\
            0 \\
            0 \\
            0.608696 \\
            -1.56522 \\
            0.521739
        \end{pmatrix}
$$

$$
    \textbf{K} = \begin{pmatrix}
        3.0385 \\
        -589.2321 \\
        684.6339 \\
        11.0901 \\
        -13.5135 \\
        95.9474
    \end{pmatrix}
$$

__Implementation in Flow*__

Flow* provides the functions and classes necessary to both incorporate the ODEs and the control law of the system and build a testing routine in C++. The first step is to create a Flow* Variables object and to define the necessary variables for the system.

```
    // Declare variables
    Variables vars;
    int x_id = vars.declareVar("x");
    int theta1_id = vars.declareVar("theta1");
    int theta2_id = vars.declareVar("theta2");
    int v_id = vars.declareVar("v");
    int omega1_id = vars.declareVar("omega1");
    int omega2_id = vars.declareVar("omega2");
    int u_id = vars.declareVar("u");
```

The ODE's that model the target system must be provided as a vector of strings. Firstly, we define some system parameters as chose above:

```
    // System parameters
    double M = 1.5; // mass of cart in kg
    double m1 = 0.5; // mass of first arm in kg
    double m2 = 0.5; // mass of second arm in kg
    double L1 = 0.5; // length of first arm in meters
    double L2 = 0.5; // length of second arm in meters
    double g = 9.8; // gravitational acceleration in meters per second squared
```

Due to the complexity of of the ODE, we define intermediary strings. These match the defintion of d1-d2 and f1-f2 above:

```
    // Matrix oefficient expressions
    string d1 = to_string(M + m1 + m2);
    string d2 = to_string((0.5*m1 + m2)*L1);
    string d3 = to_string(0.5*m2*L2);
    string d4 = to_string(((1.0/3.0)*m1 + m2)*L1*L1);
    string d5 = to_string(0.5*m2*L1*L2);
    string d6 = to_string((1.0/3.0)*m2*L2*L2);

    string f1 = to_string((0.5*m1 + m2)*L1*g);
    string f2 = to_string(0.5*m2*L2*g);
```

The $\textbf{D}$, $\textbf{C}$, $\textbf{G}$, and $\textbf{H}$ matrices are constructed, matching their definition above.

```
// Define ODEs
    string D_c = "(1/("+d1+"*"+d4+"*"+d6+" - "+d2+"^2*"+d6+"*cos(theta1)^2 - "+d1+"*"+d5+"^2*cos(theta1-theta2)^2 + 2*"+d2+"*"+d3+"*"+d5+"*cos(theta1)*cos(theta1-theta2)*cos(theta2) - "+d3+"^2*"+d4+"*cos(theta2)^2))";
    string D_11 = "("+D_c+"*("+d4+"*"+d6+" - "+d5+"^2*cos(theta1-theta2)^2))";
    string D_12 = "("+D_c+"*(-"+d2+"*"+d6+"*cos(theta1) + "+d3+"*"+d5+"*cos(theta1-theta2)*cos(theta2)))";
    ...
```

The control law is input as a separate vector string. The gains of the controller, k1-k6 correspond to the six state variables of the DIPC, and are computed based on the $\textbf{A}$ and $\textbf{B}$ matrices via a matlab script.

```
    // Define control law u
    vector<string> ctrl_law = {"(-"+K1+"*x - "+K2+"*theta1 - "+K3+"*theta2 - "+K4+"*v - "+K5+"*omega1 - "+K6+"*omega2)"};
```

Finally, we create a *feedback* object, with the above variables, ode of the system, and control law.

    // Create Feedback object
    Feedback<Real> feedback(vars, 0.0005, ode, ctrl_law);
```

---

## 4. Debugging and Challenges

The DIPC implementation in FLow* poses a few challenges. Unlike the single pendulum system, the double pendulum system simulation often terminates early with the message “Flowpipe computation terminated due to large overestimation”. The taylor model is an __overapproximation__ of the actual reachable set over time. These overapproximations amount to a small error that accumulate over time. When this error accumulate past a defined *cuttoff_threshold* value, then the simulation is terminated with this message.

<div style="text-align: center;">      
<img src="/assets/pendulum/Blowup.png" alt="Visual of blowup behavior" width="410">                                                                                                                                       
</div>  

The example above shows the state space of $\theta_0$ and $\theta_1$ across time. Both graphs are taken from a single execution with the same initial set, but shows the difference in the size of the overapproximated bounds at $T=4$ vs $T\approx 5.8$. This overapproximation “blowup” is largely unavoidable for the DIPC model due to its complex nature. Even very similar initial points will result in drastically different execution paths, diverging more and more as time increases. Because the flowpipe polygon will have to be drawn to encompass all possible execution paths, the diverging nature will quickly lead to blowups. This eliminates our ability to define an initial set for DIPC simulations. Instead we start from the smallest possible set: a point. however, even from a point the simulation overapproximation still exists, since Flow* automatically sets a tiny interval around the initial point.

There are a few strategies one can use to minimize the overapproximation blowup as much as possible. Higher order of taylor series approximations and smaller stepsizes of Flow* simulations both act to improve the accuracy of the simulation and thus reduce the overapproximation error. This approach does delay the blowing up behavior, but it isn’t eliminated; moreover, the computation time increases very quickly in an exponential manner. The figure below compares the result of taylor order 2 vs 3. The less precise TM=2 simulation blows up much sooner.

<div style="text-align: center;">      
<img src="/assets/pendulum/overlay.png" alt="taylor order 2 vs 3 comparison" width="410">                                                                                                                                       
</div>  

Based on our preliminary testing we developed the following methodology to complete our verification. The double pendulum model is limited to verification runs within $T = 4s$. This timeframe allows most models with taylor order 3 to complete without overapproximation issues. These simulations complete in about 4-6 minutes, and serve as a good tradeoff between simulation accuracy and computation cost.

Since we cannot define a larger initial set, we run a reachability check on a finite set of distinct and representative points within the wider initial set. We define an initial point as a verified safe initial point for the DIPC if the simulation is able to complete without “termination due to large overestimation”. 

---

## 5. Results

**Angle Offset, Indentical**

We wanted to test a range of different conditions for the DIPC system, so first we decided to run some tests with $\theta_1^{(0)} = \theta_2^{(0)}$. We ran lots of test runs with initial angles between -0.8 and 0.8. A few of the resulting graphs are shown below.

<div style="text-align: center;">      
<img src="/assets/pendulum/d_0.2.png" alt="simulation" width="450">                                                                                                                                       
</div>  

We can see from the above figures that the system follows a similarly shaped trajectory for the angles in this range. The overall domain of values varies depending on the size of the initial angle, and the motion for negative and positive angles are diagonally mirrored, but the overall shape of the trajectories are very similar.

<div style="text-align: center;">      
<img src="/assets/pendulum/d_0.8.png" alt="simulation" width="410">                                                                                                                                       
</div>  

The above graphs depict a failure, where the simulation ended very early on due to overestimation. The cart position and velocity begin to take off, and it fails to bring the angles closer to 0 before the simulation ends. From further testing, we concluded that the controller is safe with an initial angle range of [-0.70, 0.70] up to a time horizon of T = 4.0 seconds, according to our criteria.


**Angle Offset, Differing**




---

## 9. Lessons Learned

* Architectural insights gained
* Design mistakes and how you would correct them
* Scalability limitations discovered
* Engineering principles reinforced

---

## 10. Future Work

* Performance improvements
* Architectural redesign ideas
* Feature extensions
* Scalability enhancements
* Production-level considerations

---

## 11. Visual Documentation

Include:
* Block diagrams
* Timing diagrams
* PCB layouts
* System photos
* 3D renders
* Waveform captures
* Annotated architecture diagrams

---

## 12. Reproducibility

* Build instructions
* Setup steps
* Known limitations
* Dependencies
* How to replicate results