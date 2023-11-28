# Controller Library

This library is associated with creating controllers for robotic systems.

## Basic Controller Class


## Operational Space Control (OSC) Class

The OSC class is concerned with creating control outputs $u$ that satisfy the dynamics of the system $M \ddot{q} + h = B u + J_c^T \lambda_c$ whilst minimising the error of tracking tasks $x_i(q)$. This class solves these problems through the following quadratic program

$$
\begin{align}
u \in \arg\min_{\ddot{q},u,\lambda_c} & \sum_i w_i || J_i \ddot{q} + \dot{J}_i \dot{q} + \ddot{x}_d ||^2 + w_u || u ||^2\\
\textrm{s.t.} \quad & M(q) \ddot{q} + h(q,\dot{q}) = B(q) u + J_c^T \lambda_c\\
& \lambda_c \in \mathcal{F}_c\\
& u\_{\min} \le u \le u\_{\max}\\
\end{align}
$$
