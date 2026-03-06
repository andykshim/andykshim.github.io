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

This represents the total accumulated cost for every state $ \textbf{x}_k $ and control $ \textbf{u}_k $ from time $ t $ to $ t_{final} $. $L_k$ in this case corresponds to Linear Quadratic cost:

$$ L_k(\textbf{x}_k,\textbf{u}_k) = \textbf{x}_k^T\textbf{Qx}_k+\textbf{u}_k^T\textbf{Ru}_k $$ 

Here, the $ \textbf{Q} $ matrix represents the cost of deviation from the desired state, and the $ \textbf{R} $ matrix represents the cost of controller input. Both are user-defined. The dynamics equations for SIPC and DIPC can then be linearized around \textbf{x}=\textbf{0} as such:
$$ \dot{\textbf{x}}=\textbf{Ax}+\textbf{B}u $$

To implement digital control, matrices A and B are approximately discretized into $\boldsymbol\Phi\approx e^{\textbf{A}\Delta t}$ and $\boldsymbol\Gamma\approx\textbf{B}\Delta t$. Optimal digital LQR control is then given by:

$$    u_k = -\textbf{R}^{-1}\boldsymbol\Gamma^T\textbf{Px}_k\equiv-\textbf{Kx}_k $$


The \textbf{P} matrix is found by solving the discrete-time algebraic Riccati equation:
\begin{gather*}
    \boldsymbol\Phi^T[\textbf{P} - \textbf{P}\boldsymbol\Gamma(\textbf{R}+\boldsymbol\Gamma^T\textbf{P}\boldsymbol\Gamma)^{-1}\boldsymbol\Gamma^T\textbf{P}]\boldsymbol\Phi-\textbf{P}+\textbf{Q}=\textbf{0}
\end{gather*}

\paragraph{LQR for SIPC and DIPC}
For the SPIC, we define \textbf{Q} and \textbf{R} as such:
\begin{gather*}
    \begin{aligned}
        \textbf{Q} &= \begin{pmatrix}
            1 & 0 & 0 & 0 \\
            0 & 1 & 0 & 0 \\
            0 & 0 & 10 & 0 \\
            0 & 0 & 0 & 100
        \end{pmatrix} \\
        \textbf{R} &= 1
    \end{aligned}
\end{gather*}

We then linearize the \textbf{A} and \textbf{B} matrices as such \cite{BruntonSIPC}:
\begin{gather*}
    \begin{aligned}
        \textbf{A} &= \begin{pmatrix}
            0 & 1 & 0 & 0 \\
            0 & -\frac{\delta}{M} & \frac{mg}{M} & 0 \\
            0 & 0 & 0 & 1 \\
            0 & -\frac{\delta}{ML} & -\frac{(m+M)g}{ML} & 0
        \end{pmatrix} \\
        \textbf{B} &= \begin{pmatrix}
            0 \\
            \frac{1}{M} \\
            0 \\
            \frac{1}{ML}
        \end{pmatrix}
    \end{aligned}
\end{gather*}

This gives us numerical matrices:
\begin{gather*}
    \begin{aligned}
        \textbf{A} &= \begin{pmatrix}
            0 & 1 & 0 & 0 \\
            0 & -1 & 0.981 & 0 \\
            0 & 0 & 0 & 1 \\
            0 & -2 & 21.582 & 0
        \end{pmatrix} \\
        \textbf{B} &= \begin{pmatrix}
            0 \\
            1 \\
            0 \\
            2
        \end{pmatrix}
    \end{aligned}
\end{gather*}
And from here, we can calculate our K matrix:
\begin{gather*}
    \textbf{K} = \begin{pmatrix}
        -1.0000 \\
        -3.7944 \\
        39.2831 \\
        13.1990
    \end{pmatrix}
\end{gather*}

For DPIC, we define \textbf{Q} and \textbf{R} as such:
\begin{gather*}
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
\end{gather*}
We can linearize \textbf{A} and \textbf{B} as such \cite{DIPCShort}:
\begin{gather*}
    \begin{aligned}
        \textbf{A} &= \begin{pmatrix}
            \textbf{0} & \textbf{I} \\
            -\textbf{D}(\textbf{0})^{-1}\frac{\partial\textbf{G}(\textbf{0})}{\partial\theta} & \textbf{0}
        \end{pmatrix} \\
        \textbf{B} &= \begin{pmatrix}
            \textbf{0} \\
            \textbf{D}(\textbf{0})^{-1}\textbf{H}
        \end{pmatrix}
    \end{aligned}
\end{gather*}
This yields:
\begin{gather*}
    \begin{aligned}
        \textbf{A} &= \begin{pmatrix}
            0 & 0 & 0 & 1 & 0 & 0 \\
            0 & 0 & 0 & 0 & 1 & 0 \\
            0 & 0 & 0 & 0 & 0 & 1 \\
            0 & -5.75217 & 0.63913 & 0 & 0 & 0 \\
            0 & 63.1913 & -26.8435 & 0 & 0 & 0 \\
            0 & -80.5304 & 67.7478 & 0 & 0 & 0
        \end{pmatrix} \\
        \textbf{B} &= \begin{pmatrix}
            0 \\
            0 \\
            0 \\
            0.608696 \\
            -1.56522 \\
            0.521739
        \end{pmatrix}
    \end{aligned}
\end{gather*}
From this, we derive our \textbf{K} matrix:
\begin{gather*}
    \textbf{K} = \begin{pmatrix}
        3.0385 \\
        -589.2321 \\
        684.6339 \\
        11.0901 \\
        -13.5135 \\
        95.9474
    \end{pmatrix}
\end{gather*}

---

## 5. Design Tradeoffs and Decision Making

* What alternative designs were considered?
* Why was the final approach selected?
* What was sacrificed? (performance, simplicity, scalability, etc.)
* Quantitative or qualitative comparison of options

---

## 6. Performance and Results

* Clock frequency achieved
* Latency (ms, cycles)
* Throughput
* Memory usage
* Power consumption (if measured)
* Accuracy (if applicable)
* Resource utilization (LUTs, BRAM, etc.)
* Benchmark comparisons

Include:
* Tables
* Plots
* Waveforms
* Measurement methodology

---

## 7. Debugging and Challenges

* Major technical roadblocks
* What failed and why
* How issues were diagnosed
* Tools used (oscilloscope, logic analyzer, gdb, waveform viewer)
* Root cause analysis
* Lessons from debugging process

---

## 8. Verification and Testing

* Unit testing strategy
* Testbench architecture
* Coverage considerations
* Corner cases evaluated
* Simulation vs hardware validation
* Formal verification (if applicable)

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