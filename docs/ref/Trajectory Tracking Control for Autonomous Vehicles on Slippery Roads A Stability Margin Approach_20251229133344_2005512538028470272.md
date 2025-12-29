RESEARCH

# Trajectory Tracking Control for Autonomous Vehicles on Slippery Roads: A Stability Margin Approach

Ziyu Song  $\cdot$  Junlong Wang  $\cdot$  Yang Liang  $\cdot$  Zhiqi Mao  $\cdot$  Haitao Ding

Received: 4 March 2025 / Accepted: 29 April 2025 / Published online: 23 May 2025

© The Author(s), under exclusive licence to Springer Nature B.V. 2025

Abstract Trajectory tracking control on slippery roads presents significant challenges for autonomous vehicles due to nonlinear tire dynamics and reduced road adhesion. This paper proposes a trajectory tracking control framework based on a stability margin approach to improve both tracking accuracy and stability in such conditions. First, by analyzing the front and rear tire slip angles in the phase plane, we identify critical saddle points and define a stability boundary for tire slip angles. A corresponding stability index is formulated to quantify how close the vehicle is to its handling limits, enabling the control strategy to dynamically balance maneuverability and stability. This stability index is then integrated into a model predictive controller (MPC), allowing adaptive weight adjustments to prioritize either precise trajectory tracking or enhanced stability. Hardware-in-the-loop (HIL) experiments demonstrate that the proposed method can effectively improve the path tracking accuracy and ensure the handling stability. Compared with the conventional MPC strategy, the adaptive MPC reduces the longitudinal speed root mean square error (RMSE) by  $65.10\%$  and  $9.38\%$  on variable speed double lane change maneuver and split friction double lane change

maneuver, the RMSE values of yaw rate and sideslip angle decreased by  $33.96\%$  and  $24.00\%$ , respectively. The results indicate its potential for real-time deployment in autonomous vehicles tracking slippery conditions.

Keywords Trajectory tracking  $\cdot$  Model predictive control  $\cdot$  Stability margin  $\cdot$  Slippery roads  $\cdot$  Autonomous vehicles

# Nomenclature

<table><tr><td>αf</td><td>Front tire slip angle</td></tr><tr><td>αr</td><td>Rear tire slip angle</td></tr><tr><td>β</td><td>Sideslip angle</td></tr><tr><td>δf</td><td>Front wheel angle</td></tr><tr><td>φ</td><td>Yaw rate</td></tr><tr><td>μ</td><td>Tire-road friction coefficient</td></tr><tr><td>a</td><td>Distance from the center of gravity (CoG) to the front axis</td></tr><tr><td>ax</td><td>Longitudinal acceleration</td></tr><tr><td>ay</td><td>Lateral acceleration</td></tr><tr><td>b</td><td>Distance from the CoG to the rear axis</td></tr><tr><td>Cf</td><td>Front tire cornering stiffness</td></tr><tr><td>Cr</td><td>Rear tire cornering stiffness</td></tr><tr><td>Fz</td><td>Normal load</td></tr><tr><td>Fx,ij</td><td>Longitudinal tire force</td></tr><tr><td>Fy,ij</td><td>Lateral tire force</td></tr><tr><td>Iz</td><td>Moment of inertia</td></tr></table>

L Wheelbase

m Vehicle mass

$N_{c}$  Control horizon

$N_{p}$  Prediction horizon

$T$  Sampling period

$V_{x}$  Longitudinal speed

$V_{y}$  Lateral speed

# 1 Introduction

Autonomous vehicles (AVs) must achieve precise trajectory tracking and maintain stability, particularly under adverse road conditions such as slippery or low adhesion roads [1-3]. These environments introduce significant challenges due to unpredictable variations in tire-road friction [4] and dynamic uncertainties [5]. Conventional trajectory tracking methods often struggle in such conditions, as they are typically designed based on the assumption of well-characterized and relatively stable vehicle dynamics [6,7]. This study aims to improve the robustness of trajectory tracking control in these challenging scenarios by introducing a stability margin approach that explicitly incorporates tire slip dynamics and phase plane stability analysis.

Extensive research has been conducted on trajectory tracking control, leveraging various methodologies such as model predictive control (MPC) [8-10], sliding mode control (SMC) [11,12] and reinforcement learning-based [13,14] approaches. Ghandriz [15] developed an MPC approach robust to driver model uncertainties. However, linear MPC relies on local linearization and loses accuracy when tire slip angles grow large on slippery roads. Xu [16] proposed a nonlinear MPC (NMPC) for race cars, demonstrating high accuracy in aggressive maneuvers. While capturing tire-force dynamics more accurately, NMPC suffers from high computational costs, making real-time deployment difficult for embedded vehicle controllers. Similarly, Mercorelli [17] integrated a velocity observer and flatness-based feedforward with MPC for an actuator, improving tracking performance albeit at the cost of increased observer complexity. As an extension of flatness-based control methods, Mercorelli [18] proposed combining flatness-based feedforward action with a fractional-order PI controller to improve trajectory tracking accuracy in valve control, though at the expense of increased design complexity. To address the modeling inaccuracies and com

putational limitations in NMPC, learning-based adaptive control strategies have also been explored. Maaruf [19] proposed a neuro-adaptive backstepping controller for autonomous ground vehicles with actuator deadzones. By employing radial basis function neural networks (RBFNNs) with minimum parameter learning and command filters, their method achieves robust path following with reduced computational complexity. This demonstrates the potential of combining neural approximators with structured control design for real-time deployment under uncertainties. To enhance MPC's capability in complex, dynamic scenarios, recent studies have explored hierarchical and cloud-based predictive frameworks. Dong [20] introduced an overtaking-enabled eco-approach control strategy based on MPC, where the controller dynamically plans longitudinal trajectories under traffic constraints. Their work demonstrates how MPC can be extended to incorporate interactive driving behavior. Further, Dong [21] proposed a predictive strategy that leverages a two-layer MPC structure to handle long-horizon constraints and short-horizon dynamics simultaneously. These studies highlight MPC's flexibility in integrating multiple objectives and time scales, motivating its application in our work for trajectory tracking under low-adhesion conditions, where stability margin guidance is used to adapt control weights in real-time.

Beyond MPC-based approaches, SMC and fuzzy control strategies have also been increasingly adopted in trajectory tracking applications for their robustness in uncertain and nonlinear environments. For instance, Yin [22] formulated a robust decoupling control strategy, ensuring lateral stability through a four-wheel steering system. However, robust control methods are designed to handle worst-case scenarios, which often leads to conservative performance. Song [23] further employed a sliding mode-based stability controller, demonstrating robustness under variable road conditions. However, SMC often introduces high-frequency oscillations in control inputs, reducing actuator lifespan and affecting ride comfort. Ding [24] proposed a hierarchical control framework integrating trajectory generation and trajectory tracking with a fuzzy PID controller, which demonstrates reliable path tracking. However, the approach lacks explicit handling of tire-road interaction limits or dynamic stability constraints that are critical for autonomous ground vehicles operating on slippery or low-adhesion roads. Mercorelli [25] proposed a fuzzy-based control architecture for robots,

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/fd524562c93030981d4202b403c929a1d64411f13012353f97b386470041bc86.jpg)  
Fig. 1 Overall framework of trajectory tracking control

integrating an obstacle avoidance mechanism through a cascaded fuzzy-PD structure. Their results demonstrated improved stability and reduced steering effort during evasive maneuvers. Similarly, Mercorelli [26] developed a fuzzy PD control strategy combined with a Luenberger observer to achieve soft trajectory following in the presence of obstacles, further confirming the effectiveness of fuzzy logic in dynamic path planning tasks. Mohammadzadeh [27] introduced a fractional-order type-3 fuzzy control framework for driverless cars, capable of handling severe driving conditions with unknown nonlinear dynamics and parametric uncertainties. Their controller integrates fuzzy estimation with a predictive lateral displacement enhancement mechanism and demonstrates superior performance under varied tire-road interactions. Building upon these developments, we integrate a fuzzy logic-based stability margin into our adaptive MPC controller to better handle low-adhesion road conditions.

Moreover, He [28] applied deep reinforcement learning (DRL) for lateral control, showing adaptability to unknown friction levels. Similarly, Han [29] integrates pure pursuit with proximal policy optimization (PPO)-based DRL, employing an adaptive look-ahead mechanism to dynamically adjust preview distances based on speed and road curvature, thereby improving tracking accuracy and robustness under complex driving conditions. Recently, Liang [30] proposed an integrated DRL framework that unifies behavior planning, trajectory generation, and motion control in high-speed cruising scenarios. Their work leverages Boot

strapped DQN for behavioral decisions and MPC for trajectory tracking, their approach addresses stability and feasibility issues often overlooked in modular systems, demonstrating the adaptability and physical consistency of learning-based control in time-critical scenarios. RL-based controllers lack explainability and require extensive training data covering diverse road conditions, which is often impractical in real-world deployments.

The phase plane method is the most used and effective method for stability analysis. The phase planes for vehicle dynamics analysis can be categorized into three pairs of states variables: front and rear tire slip angles  $(\alpha_{f} - \alpha_{r})$ , sideslip angle and yaw rate  $(\beta - \dot{\varphi})$ , and sideslip angle and sideslip angle rate  $(\beta - \dot{\beta})$ . In some studies, the phase plane has also been utilized for controller design. Beal [31] proposed a safety envelope bounded by the saturation rear tire slip angle and the min/max steady-state yaw rate to restrict the vehicle state to a stability region in the  $\beta - \dot{\varphi}$  phase plane. Li [32] used the same real-time envelope scheme to distinguish between stable, critical stable and unstable states for coordinated control of electric vehicles using MPC. Chen [33] used the stability envelope scheme via  $\beta - \dot{\varphi}$  phase plane as a stability constraint for coordinated control. Although the  $\beta - \dot{\varphi}$  phase plane can describe the overall stability of the vehicle, it cannot directly reflect whether the tire slip angle enters the nonlinear region, it may not be accurate enough to judge the vehicle state under slippery roads. Moreover, Zhou [34] proposed a controller based on the closed self-stable region constructed from the yaw rate

constraint and the  $\beta$ - $\dot{\beta}$  phase plane, which improved vehicle handling stability according to the simulation and experimental results. Liang [35] introduced the  $\beta$ - $\dot{\beta}$  phase plane constraint into the integrated control scheme of active steering and direct yaw moment control (DYC) systems for better balancing maneuverability and stability. Slippery roads often involve significant input disturbances, such as driver steering inputs and external interferences, the calculation of  $\dot{\beta}$  is highly sensitive to noise under these conditions, which may lead to unstable control behavior. In contrast,  $\alpha_{f}-\alpha_{r}$  are directly derived from the tire model and have lower measurement errors, preventing control failures caused by noise amplification. Cairano [36] introduced an  $\alpha_{f}-\alpha_{r}$  phase plane constraint in the design of a model predictive stability controller to ensure stability. Similarly, Li [37] proposed a stability evaluation method and a stability index based on the  $\alpha_{f}-\alpha_{r}$  phase plane and designed an adaptive SMC scheme that can effectively improve vehicle handling stability. The  $\alpha_{f}-\alpha_{r}$  phase plane can detect excessive tire side angle and insufficient adhesion earlier, which is more advantageous on low adhesion roads. Although the  $\alpha_{f}-\alpha_{r}$  phase plane offers a more physically grounded and noise-resilient representation, most prior works rely on fixed threshold boundaries or single-index evaluation, which may not capture the full dynamics of stability degradation.

Given the limitations of the above methods, there is a critical need for a control framework that (1) explicitly considers stability margins by accounting for tire slip saturation and nonlinear dynamics, (2) dynamically adjusts control priorities between tracking accuracy and stability, and (3) remains computationally efficient for real-time implementation. To address these issues, in this paper, we propose a stability margin approach that systematically incorporates tire slip dynamics into trajectory tracking control. Unlike conventional MPC or adaptive controllers, our method uses a vehicle model to define a phase plane stability boundary for tire slip angles. We further identify saddle points in the  $\alpha_{f} - \alpha_{r}$  phase plane, defining a stability region that reflects how close the vehicle is to instability. We integrate this stability index into MPC, which dynamically adjusts weightings between tracking accuracy and stability preservation. The main contributions are as follows:

1. A more accurate and physically meaningful stability margin method is developed by analyzing the  $\alpha_{f} - \alpha_{r}$

phase plane behavior. Unlike prior works that rely on fixed thresholds or single-index criteria, our method captures the dynamic evolution of stability boundaries using a dual-index structure, enabling robust performance under varying adhesion conditions.

2. A stability-aware MPC formulation is introduced, where control weights are adaptively adjusted based on real-time stability indices. This expands upon fixed-weight strategies and improves the controller's ability to trade off between tracking performance and vehicle stability depending on driving risk.  
3. The proposed control framework is tested in real-time HIL experiments, demonstrating improved tracking precision and stability on slippery roads compared to traditional MPC methods.

The detailed architecture is illustrated in Fig. 1. The remainder of the paper is as follows: Section 2 presents the stability margin and index methodology based on phase plane analysis, and Section 3 describes the controller design, incorporating the stability margin into an adaptive MPC framework. Subsequently, Section 4 details the HIL experiment setup and comparative evaluations, followed by the key conclusions summarized in Section 5.

# 2 Stability index methodology

This section examines the effects of speed, front wheel steering angle, and tire-road friction coefficient on the saddle point positions, an equation for the saddle-point slip angle is derived from a three-degree-of-freedom (3-DOF) vehicle dynamics model. The equation parameters are estimated using the least squares method, facilitating the definition of the stability margin. Furthermore, by analyzing the relationship between the vehicle state, the tire saturation slip angle, and the tire slip angle stability boundary, a theoretical model for stability index is proposed. This model provides a foundation for future research on vehicle trajectory tracking control strategies on low adhesion roads.

# 2.1 Design of phase plane

As shown in Fig. 2, the 3-DOF model [38] effectively balances model fidelity with computational complexity, making it suitable for stability analysis. The model assumes small front wheel steering angles and neglects

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/ea97c892aa734f2db00554c5b6c0a4f56f611ac0a91b994d6a24bf84a91e58b8.jpg)  
Fig. 2 3-DOF vehicle dynamics model

the differences between the left and right wheels and variations in tire characteristics caused by load changes between the left and right sides of the vehicle, the longitudinal speed is assumed to be constant, and the pitch and roll motion are neglected. The expression for the sideslip angle rate [38] is:

$$
\begin{array}{l} \dot {\beta} = \frac {F _ {x f} \cdot \sin \delta_ {f} + F _ {y f} \cdot \cos \delta_ {f} + F _ {y r}}{m V _ {x}} \\ - \frac {\left(a _ {x} + V _ {y} \dot {\varphi}\right) \cdot \beta}{V _ {x}} - \dot {\varphi} \tag {1} \\ \end{array}
$$

Assuming that small sideslip angles, the expression simplifies to:

$$
\dot {\beta} = \frac {F _ {x f} \cdot \sin \delta_ {f} + F _ {y f} \cdot \cos \delta_ {f} + F _ {y r}}{m V _ {x}} - \dot {\varphi} \tag {2}
$$

The first derivative of the yaw rate is given by:

$$
\ddot {\varphi} = \frac {\left(F _ {x _ {f}} \cdot \sin \delta_ {f} + F _ {y _ {f}} \cdot \cos \delta_ {f}\right) a - F _ {y r} \cdot b}{I _ {z}} \tag {3}
$$

The slip angles of the front and rear tires are respectively are defined as:

$$
\left\{ \begin{array}{l} \alpha_ {f} = \arctan \left(\beta + \frac {a}{V _ {x}} \dot {\varphi}\right) - \delta_ {f} \approx \beta + \frac {a}{V _ {x}} \dot {\varphi} - \delta_ {f} \\ \alpha_ {r} = \arctan \left(\beta - \frac {b}{V _ {x}} \dot {\varphi}\right) \approx \beta - \frac {b}{V _ {x}} \dot {\varphi} \end{array} \right. \tag {4}
$$

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/03b9b17919b517ce381cc74f61a2a1c01e5b6bc463742fe16b9a2b09273a3610.jpg)  
Fig.3  $\alpha_{f} - \alpha_{r}$  phase plane

By combining the above equations, we derive the first-order differential equations for the slip angles of the front and rear tires as follows:

$$
\begin{array}{l} \dot {\alpha_ {f}} = \frac {F _ {x f} \sin \delta_ {f} + F _ {y f} \cos \delta_ {f} + F _ {y r}}{m V _ {x}} \\ - \frac {\left(\alpha_ {f} - \alpha_ {r} + \delta_ {f}\right) V _ {x}}{L} \\ + \frac {a \left(\left(F _ {x _ {f}} \sin \delta_ {f} + F _ {y _ {f}} \cos \delta_ {f}\right) a - F _ {y r} b\right)}{I _ {z} V _ {x}} \\ \end{array}
$$

$$
\begin{array}{l} \dot {\alpha_ {r}} = \frac {F _ {x f} \sin \delta_ {f} + F _ {y f} \cos \delta_ {f} + F _ {y r}}{m V _ {x}} \\ - \frac {\left(\alpha_ {f} - \alpha_ {r} + \delta_ {f}\right) V _ {x}}{L} \\ - \frac {b \left(\left(F _ {x _ {f}} \sin \delta_ {f} + F _ {y _ {f}} \cos \delta_ {f}\right) a - F _ {y r} b\right)}{I _ {z} V _ {x}} \tag {5} \\ \end{array}
$$

Based on Eq. 5, we analyze how longitudinal speed, front-wheel steering angle, and friction coefficient affect the stability margin. By assigning different initial values to these factors and solving the differential equations for the front and rear tire slip angles, we obtain the  $\alpha_{f} - \alpha_{r}$  phase plane, the lateral tire force is computed using the Magic tire model. As shown in Fig. 3. The points  $S_{1}(\alpha_{f1},\alpha_{r1})$  and  $S_{2}(\alpha_{f2},\alpha_{r2})$  represent the saddle points, and the stable region in the  $\alpha_{f} - \alpha_{r}$  phase plane is defined as the circle with  $S_{1}$  and  $S_{2}$  as its diameter.

Furthermore, by examining the effects of longitudinal speed, front wheel steering angle, and friction

coefficient on the dynamic characteristics, this analysis provides a foundation for the subsequent stability margin design. As shown in Fig. 4(a), when  $\delta_f = 0$ ,  $\mu = 1$ , and  $V_x$  increases from  $30\mathrm{km/h}$  to  $90\mathrm{km/h}$ , the stable equilibrium point gradually converges near the origin. The unstable left and right saddle points exhibit similar behavior, both moving toward the stable equilibrium point, causing the stable region in the  $\alpha_f - \alpha_r$  phase plane to shrink rapidly. From a dynamics perspective, with the same friction coefficient, the tire adhesion limit remains unchanged. However, as speed increases, the longitudinal force demand increases, reducing the lateral force available for stability. Consequently, the stability range of the front and rear tire slip angles decreases, making it easier for lateral forces to reach saturation. As shown in Fig. 4(b), when  $V_x = 60\mathrm{km/h}$ ,  $\mu = 1$ , and  $\delta_f$  increases from 0 deg to 8 deg, the stable equilibrium point no longer remains near the origin but gradually shifts toward the lower-left corner of the  $\alpha_f - \alpha_r$  phase plane. The left saddle point moves toward the stable equilibrium point, and when  $\delta_f = 8$  deg, the left saddle point almost coincides with the stable equilibrium point. In contrast, the right saddle point gradually moves away from the stable equilibrium point. However, the size of the stable region in the phase plane remains largely unchanged, with only the positions of the stable equilibrium point and unstable saddle points shifting. As shown in Fig. 4(c), when  $V_x = 60\mathrm{km/h}$ ,  $\delta_f = 0$ , and  $\mu$  decreases from 0.8 to 0.2, the stable equilibrium point remains near the origin, while the unstable left and right saddle points both move closer to the stable equilibrium point. As a result, the stable region in the  $\alpha_f - \alpha_r$  phase plane shrinks rapidly. Since a lower  $\mu$  reduces the tire adhesion limit, the available lateral force decreases when the longitudinal driving force remains constant. Consequently, the stability range of the front and rear tire slip angles decreases, and the positions of the unstable saddle points are inversely proportional to the friction coefficient.

# 2.2 Design of stability margin

Based on the dynamic relationships among the yaw rate, sideslip angle, and tire slip angles, the slip angles at the saddle points can be determined from the yaw rate and sideslip angle at these points. The first is to establish the stability boundary for the yaw rate and sideslip angle. The stability margin model is derived

in three steps: (1) determine yaw rate boundaries, (2) derive sideslip angle boundaries at tire saturation, and (3) relate these to tire slip angle boundaries.

1) Stability boundary of yaw rate

According to the 3-DOF model, the lateral acceleration is given by:

$$
a _ {y} = \dot {V} _ {y} + V _ {x} \dot {\varphi} = \dot {V} _ {x} \cdot \tan \beta + \frac {V _ {x} \cdot \dot {\beta}}{\sqrt {1 + \tan^ {2} \beta}} + V _ {x} \dot {\varphi} \tag {6}
$$

Since the maximum tire force provided by the road is  $\mu mg$ , the lateral acceleration satisfies:

$$
a _ {y} = \dot {V} _ {x} \cdot \tan \beta + \frac {V _ {x} \cdot \dot {\beta}}{\sqrt {1 + \tan^ {2} \beta}} + V _ {x} \dot {\varphi} \leq \mu g \tag {7}
$$

Assuming that sideslip angles and their derivatives are sufficiently small, the first and second terms in Eq. 7 can be neglected. The maximum and minimum steady-state yaw rates [39] are given by:

$$
\left\{ \begin{array}{l} \varphi_ {s, \max } = \frac {\mu g}{V _ {x}} \\ \varphi_ {s, \min } = - \frac {\mu g}{V _ {x}} \end{array} \right. \tag {8}
$$

2) Stability boundary of sideslip angle

At the saddle point, the vehicle typically reaches its maximum steady-state lateral acceleration, with the front and rear tire slip angles approaching saturation. Thus, the stability boundary of the sideslip angle is derived from the dynamic relationship between the saturated slip angles of the front and rear tires and the sideslip angle.

Substituting the saturated slip angles  $\alpha_{f,s}$  and  $\alpha_{r,s}$  into Eq. 4 yields the stability boundary expressions for the sideslip angle:

$$
\left\{ \begin{array}{l} \beta_ {\alpha_ {f, \max }} = \tan (\delta_ {f} + \alpha_ {f, s}) - \frac {a}{V _ {x}} \dot {\varphi} \\ \beta_ {\alpha_ {f, \min }} = \tan (\delta_ {f} - \alpha_ {f, s}) - \frac {a}{V _ {x}} \dot {\varphi} \\ \beta_ {\alpha_ {r, \max }} = \tan (\alpha_ {r, s}) + \frac {b}{V _ {x}} \dot {\varphi} \\ \beta_ {\alpha_ {r, \min }} = \tan (- \alpha_ {r, s}) + \frac {b}{V _ {x}} \dot {\varphi} \end{array} \right. \tag {9}
$$

Therefore, the factors primarily affecting the stability boundary of the sideslip angle include longitudinal speed, front wheel steering angle, and road friction coefficient. However, the stability boundary derived from the rear tire's saturated slip angle is independent

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/d0feed754254973e7c69c734bf099fbccd6d892ac7d73d1eabb21382f44aa861.jpg)  
Fig. 4 Analysis of stability boundaries in the phase plane. (a) Phase plane changes with different  $V_{x}$ . (b) Phase plane changes with different  $\delta_f$ . (c) Phase plane changes with different  $\mu$

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/b864affa55c79c7324e3fad369a8ea39a83c14c8c3640fe38d51c379d0079ab8.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/dad3c2ae18aa548145ba0ac88fc51f4a7de9f6b2f0d8ab03f14a0b5abffd50f2.jpg)

of the front wheel steering angle. To further determine the stability boundary of the sideslip angle under different vehicle states and road conditions, as shown in Fig. 5 plots the steady-state yaw rate and the sideslip angle stability boundaries at  $v = 60 \mathrm{~km/h}$  and  $\mu = 1$ , for different front wheel angles. In the graph, the black line represents the steady-state boundary of the yaw rate  $\varphi_s$ , the red line indicates the stability boundary of the sideslip angle  $\beta_{\alpha_f}$ , derived from the front tire slip angle and the sideslip angle, while the blue line represents the stability boundary of the sideslip angle  $\beta_{\alpha_r}$ , derived from the rear tire slip angle and the sideslip angle. The hollow circles denote the minimum steady-state sideslip angle  $\beta_{s,min}$ , and the solid circles represent the maximum steady-state sideslip angle  $\beta_{s,max}$ .

As the front wheel steering angle increases, the slope of the stability boundary for the sideslip angle, derived from the front tire slip angle, remains constant; however, the intersection point with the steady-state yaw rate boundary continuously moves to the right. The stability boundary for the sideslip angle derived from the rear tire slip angle is independent of the front wheel steering angle, and therefore, both its slope and the intersection point with the steady-state yaw rate boundary remain unchanged.

Fig. 6 illustrates the variation of  $\beta_{\alpha_f}$  and  $\beta_{\alpha_r}$  under different front wheel steering angles. The determination of the minimum stability boundary for the sideslip angle depends on the front wheel steering angle. When the front wheel steering angle is less than 6 degrees, the sideslip angle at the intersection of the  $\beta_{\alpha_f}$  curve with the maximum steady-state yaw rate curve is smaller than that at the intersection of the  $\beta_{\alpha_r}$  curve with the

maximum steady-state yaw rate curve. In this case, the minimum stability boundary for the sideslip angle is determined by the intersection of the  $\beta_{\alpha_r}$  curve with the maximum steady-state yaw rate curve. At a front wheel steering angle of 6 degrees, the  $\beta_{\alpha_f}$  curve and the  $\beta_{\alpha_r}$  curve intersect with the maximum steady-state yaw rate curve at the same point. When the front wheel steering angle exceeds 6 degrees, the sideslip angle at the intersection of the  $\beta_{\alpha_f}$  curve with the maximum steady-state yaw rate curve becomes larger than that at the intersection of the  $\beta_{\alpha_r}$  curve with the maximum steady-state yaw rate curve, thereby determining the minimum stability boundary for the sideslip angle at this intersection. The maximum stability boundary of the sideslip angle is independent of the front wheel steering angle. The sideslip angle at the intersection of the  $\beta_{\alpha_f}$  curve with the maximum steady-state yaw rate curve is always larger than that of the  $\beta_{\alpha_r}$  curve. Therefore, the minimum stability boundary is always determined by the intersection of the  $\beta_{\alpha_r}$  curve with the maximum steady-state yaw rate curve.

The critical front wheel steering angle  $\delta_{f,s}$ , which influences the minimum stability boundary of the sideslip angle, is defined as the critical point. By simultaneously solving Eq. 8 and Eq. 9, the expression for the critical front wheel steering angle  $\delta_{f,s}$  can be derived as follows:

$$
\delta_ {f, s} = \alpha_ {f, s} + \arctan \left(\frac {(a + b) \cdot \varphi_ {s , \max}}{V _ {x}} - \tan (\alpha_ {r, s})\right) \tag {10}
$$

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/a4ec0c30791d82669c78e47c1c1153de7391cf633a08d51952b95686f0b98dde.jpg)  
Fig. 5 Steady-state boundary variation of sideslip angle under different front wheel angles.  $V_{x} = 60 \, \mathrm{km/h}$ ,  $\mu = 1$ . (a)  $\delta_{f} = 0 \, \mathrm{deg}$ . (b)  $\delta_{f} = 2 \, \mathrm{deg}$ . (c)  $\delta_{f} = 4 \, \mathrm{deg}$ . (d)  $\delta_{f} = 6 \, \mathrm{deg}$ . (e)  $\delta_{f} = 8 \, \mathrm{deg}$ . (f)  $\delta_{f} = 10 \, \mathrm{deg}$

Thus, the expressions for the maximum steady-state sideslip angle  $\beta_{s,\mathrm{max}}$ , and the minimum steady-state sideslip angle  $\beta_{s,\mathrm{min}}$ , can be derived as follows:

$$
\begin{array}{l} \beta_ {s, \max } = \tan \left(\alpha_ {r, s}\right) + \frac {b}{V _ {x}} \varphi_ {s, \min } \\ \beta_ {s, \min } = \left\{ \begin{array}{l} \tan (- \alpha_ {r, s}) + \frac {b}{V _ {x}} \varphi_ {s, \max } \delta_ {f} \leq \delta_ {f, s} \\ \tan (\delta_ {f} - \alpha_ {f, s}) - \frac {a}{V _ {x}} \varphi_ {s, \max } \delta_ {f} > \delta_ {f, s} \end{array} \right. \tag {11} \\ \end{array}
$$

3) Stability boundary of tire slip angle

Since the front wheel steering angle and friction coefficient affect the yaw rate stability boundary, adjustment functions  $f_{1}(\delta_{f},\mu)$  and  $f_{2}(\delta_{f},\mu)$  are introduced. The yaw rate at the saddle point positions on the  $\alpha_{f} - \alpha_{r}$  phase plane is given by:

$$
\begin{array}{l} \dot {\varphi} _ {s 1} = \frac {\mu g}{V _ {x}} + f _ {1} (\delta_ {f}, \mu) \\ \dot {\varphi} _ {s 2} = - \frac {\mu g}{V _ {x}} + f _ {2} (\delta_ {f}, \mu) \\ \end{array}
$$

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/eba0c65e89a096f9d946163e5bcb3f572da3485b44613f333736da2a9d74aead.jpg)  
Fig. 6 The variation trend of the stability boundary

The expressions for  $f_{1}(\delta_{f},\mu)$  and  $f_{2}(\delta_{f},\mu)$  are as follows:

$$
f _ {1} (\delta_ {f}, \mu) = p _ {1} + p _ {2} \cdot \delta_ {f} + \frac {p _ {3}}{\mu} + \frac {p _ {4}}{V _ {x}} \tag {13}
$$

$$
f _ {2} (\delta_ {f}, \mu) = p _ {5} + p _ {6} \cdot \delta_ {f} + \frac {p _ {7}}{\mu} + \frac {p _ {8}}{V _ {x}}
$$

Table 1 Parameters of the saddle point position equation in the  ${\alpha }_{f} - {\alpha }_{r}$  phase plane  

<table><tr><td>Parameters</td><td>Value</td><td>Parameters</td><td>Value</td><td>Parameters</td><td>Value</td><td>Parameters</td><td>Value</td><td>Parameters</td><td>Value</td></tr><tr><td>p1</td><td>-0.0071</td><td>p2</td><td>0.1636</td><td>p3</td><td>0.0057</td><td>p4</td><td>-1.0897</td><td>p5</td><td>0.0049</td></tr><tr><td>p6</td><td>0.5244</td><td>p7</td><td>-0.0046</td><td>p8</td><td>1.1208</td><td>p9</td><td>-0.7949</td><td>p10</td><td>-4.6275</td></tr><tr><td>p11</td><td>0.4366</td><td>p12</td><td>32.6700</td><td>p13</td><td>-0.9695</td><td>p14</td><td>3.7984</td><td>p15</td><td>0.6107</td></tr><tr><td>p16</td><td>32.3603</td><td>p17</td><td>-2.619</td><td>p18</td><td>9.905</td><td>p19</td><td>0.0006387</td><td></td><td></td></tr></table>

Since speed, front wheel steering angle, and friction coefficient affect the sideslip angle stability boundary, adjustment functions  $f_{3}(V_{x},\delta_{f},\mu)$  and  $f_{4}(V_{x},\delta_{f},\mu)$  are introduced. The sideslip angle at the saddle point positions on the  $\alpha_{f} - \alpha_{r}$  phase plane is given by:

$$
\begin{array}{l} \beta_ {s 1} = \left\{ \begin{array}{l} \tan \left(- \alpha_ {r, s}\right) \cdot f _ {3} \left(V _ {x}, \delta_ {f}, \mu\right) + \frac {b}{V _ {x}} \dot {\varphi} _ {s 1} \\ \delta_ {f} \leq \delta_ {f, s} \\ \tan \left(\delta_ {f} - \alpha_ {f, s}\right) \cdot f _ {3} \left(V _ {x}, \delta_ {f}, \mu\right) - \frac {a}{V _ {x}} \dot {\varphi} _ {s 1} \\ \delta_ {f} > \delta_ {f, s} \end{array} \right. \\ \beta_ {s 2} = \tan (\alpha_ {r, s}) \cdot f _ {4} (V _ {x}, \delta_ {f}, \mu) + \frac {b}{V _ {x}} \dot {\varphi} _ {s 2} \tag {14} \\ \end{array}
$$

The expressions for  $f_{3}\left(V_{x},\delta_{f},\mu\right)$  and  $f_{4}\left(V_{x},\delta_{f},\mu\right)$  are as follows:

$$
\left\{ \begin{array}{l} f _ {3} \left(V _ {x}, \delta_ {f}, \mu\right) = p _ {9} + p _ {1 0} \cdot \delta_ {f} + p _ {1 1} \cdot \mu + \frac {p _ {1 2}}{V _ {x}} \\ f _ {4} \left(V _ {x}, \delta_ {f}, \mu\right) = p _ {1 3} + p _ {1 4} \cdot \delta_ {f} + p _ {1 5} \cdot \mu + \frac {p _ {1 6}}{V _ {x}} \end{array} \right. \tag {15}
$$

The saturated slip angles of the front and rear tires are influenced by the road friction coefficient and the vertical load on the tires. The expression for the saturated tire slip angles is constructed as follows:

$$
\alpha_ {i, s} = p _ {1 7} + p _ {1 8} \cdot \mu + p _ {1 9} \cdot F _ {i, z} (i = f, r) \tag {16}
$$

Based on the dynamic relationships between the front and rear tire slip angles and the yaw rate and sideslip angle, the expression for the lower-left saddle point  $S_{1}(\alpha_{f1},\alpha_{r1})$  in the  $\alpha_{f} - \alpha_{r}$  phase plane is given as follows:

$$
\left\{ \begin{array}{l} \alpha_ {f 1} = \beta_ {s 1} + \frac {a \cdot \dot {\varphi} _ {s 1}}{V _ {x}} - \delta_ {f} \\ \alpha_ {r 1} = \beta_ {s 1} - \frac {b \cdot \varphi_ {s 1}}{V _ {x}} \end{array} \right. \tag {17}
$$

The expression for the upper-right saddle point  $S_{2}(\alpha_{f2},\alpha_{r2})$  is given as follows:

$$
\left\{ \begin{array}{l} \alpha_ {f 2} = \beta_ {s 2} + \frac {a \cdot \dot {\varphi} _ {s 2}}{V _ {x}} - \delta_ {f} \\ \alpha_ {r 2} = \beta_ {s 2} - \frac {b \cdot \varphi_ {s 2}}{V _ {x}} \end{array} \right. \tag {18}
$$

Based on the positions of the saddle points  $S_{1}$  and  $S_{2}$  in the  $\alpha_{f} - \alpha_{r}$  phase plane under various states and road conditions, the parameters in Eq. 17 and Eq. 18 are identified via the least squares method [40], ensuring optimal fitting accuracy. The identification results are shown in Table 1.

By substituting the identified parameters of the saddle point position equations in the  $\alpha_{f} - \alpha_{r}$  phase plane into Eq. 17 and Eq. 18, the computed saddle point positions are obtained and compared with the real saddle point values, as shown in Fig. 7. The results demonstrate a strong agreement between the real and fitted saddle point values in the  $\alpha_{f} - \alpha_{r}$  phase plane.

# 2.3 Design of handling stability index

To further represent stability under various vehicle states and road conditions, it is necessary to establish a theoretical model for stability margin. This model primarily considers two aspects: the tire's saturated slip angle and the stable boundary of the tire slip angle.

The saturated slip angle is a critical parameter characterizing handling stability. Since the front tire's saturated slip angle is generally greater than that of the rear tire, the stability margin is divided into three regions: a circle with the rear tire's saturated slip angle as its radius defines a green area representing the safe region, denoted as  $R_{1} = \alpha_{r,s}$ . A circle with the front tire's saturated slip angle as its radius defines a yellow area representing the transition region, denoted as  $R_{2} = \alpha_{f,s}$ . The area outside both circles represents the danger region. Defining the current coordinates of the tire slip angles

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/9a2b4968a963eb442311eec617f78df4b6271cddc278f8ac164c3f288a2829eb.jpg)  
Fig. 7 Illustration of the fitting of the saddle point position in the phase plane. (a) Fitting of speed changes. (b) Fitting of front wheel steering angle changes. (c) Fitting of friction coefficient changes

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/e3bd3c98e4e40831fe5ddc9d1450182c032214dc063fd32f1cb93034c6a07c28.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/b19abdd9272e7dcdcc6e1cc36a6adaca2b6ec8bd47048c7785f1af3ba90aeb97.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/e3a5dd20e4f62b660c8d7529cc0f7edeb156e2f4a5238b01760b418a5327c91f.jpg)  
(a)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/d4613c1a88e03a691c411d3bebcd7a67e52b16351055ec0fb3a92937d35610bf.jpg)  
(b)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/3d7a09b9074965907539d84ae58e3ef6cbfaeaacc148b75917d9579e784687ea.jpg)  
(a)  
Fig. 9 (a) The index based on stability boundary of tire slip angle. (b) Region distinction according to stability index  $\xi_{2}$

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/94f310059b6011d9e2734074549216d4d7742097a2fcdfca9abb041e440d1afd.jpg)  
Fig. 8 (a) The index based on tire saturation slip angle. (b) Region distinction according to stability index  $\xi_{1}$  
(b)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/32a25002b10c04ad8caa85f73e579c145ab07c6b736f45d92c1d05ee54702afa.jpg)  
Fig. 10 Stability index model via fuzzy logic (combining  $\xi_{1}$  and  $\xi_{2}$  into final index  $\xi$ )

as  $(\alpha_{f},\alpha_{r})$ , and  $R = \sqrt{\alpha_f^2 + \alpha_r^2}$ , yields the stability margin diagram based on tire saturated slip angles, as shown in Fig. 8.

When  $R \leq R_{1}$ , both front and rear tire slip angles are below their respective saturation limits. The tire forces have not yet reached the adhesion limit, ensuring good stability. The focus is on enhancing maneuverability. When  $R_{1} \leq R \leq R_{2}$ , the rear tire slip angle exceeds its saturation limit, indicating potential instability, but the front tire retains additional force capacity, allowing some control over handling. When  $R > R_{2}$ , both front and rear tire slip angles exceed their saturation limits, meaning the vehicle reaches the adhesion limit, loses steering capability, and may experience fishtailing. The focus shifts to enhancing stability. The handling stability index is defined as:

$$
\xi_ {1} = \left\{ \begin{array}{l l} \frac {R}{2 \cdot R _ {1}} & (R \leq R _ {1}) \\ \frac {R}{2 \left(R _ {2} - R _ {1}\right)} + \frac {R _ {2} - 2 R _ {1}}{2 \left(R _ {2} - R _ {1}\right)} & (R _ {1} <   R \leq R _ {2}) \\ \frac {R}{R _ {2}} & (R > R _ {2}) \end{array} \right. \tag {19}
$$

Specifically, when  $0 < \xi_{1} \leq 0.5$ , the vehicle is stable, prioritizing maneuverability. When  $0.5 < \xi_{1} \leq 1$ , the vehicle enters the transition region, balancing maneuverability and stability. When  $1 < \xi_{1}$ , the vehicle reaches the danger region, where stability becomes the primary concern.

The stability boundary of tire slip angles is determined via phase plane analysis, which is a key indicator of handling stability. If we define a circle with the diameter spanning between two saddle points in the  $\alpha_{f} - \alpha_{r}$  phase plane as the safe region, the radius

is:  $R_{3} = 0.5\sqrt{\left(\alpha_{f1} - \alpha_{f2}\right)^{2} + \left(\alpha_{r1} - \alpha_{r2}\right)^{2}}$  and the center coordinates of the circle are  $(\alpha_{f,c},\alpha_{r,c})$ . When  $\delta_f = 0$ , as shown in Figs. 4(a) and (c), the safe region has its center at the origin, and the saddle points are symmetrically placed, represented as  $(\alpha_{f,c},\alpha_{r,c}) = (0,0)$ . When  $\delta_f \neq 0$ , as shown in Fig. 4(b), the safe region remains the same size, but its center shifts to the upper left in the phase plane. We defined the current coordinates of the safe region center as  $\left(\alpha_{f,c},\alpha_{r,c}\right) = \left(0.5\left(\alpha_{f1} + \alpha_{f2}\right),0.5\left(\alpha_{r1} + \alpha_{r2}\right)\right)$ . The current position of the tire slip angle is defined as:  $(\alpha_{f},\alpha_{r})$ , and  $R^{\prime} = \sqrt{\left(\alpha_{f} - \alpha_{f,c}\right)^{2} + \left(\alpha_{r} - \alpha_{r,c}\right)^{2}}$ . As depicted in Fig. 9, the red point represents the origin, the black point represents the center of the safe region, the green area denotes the safe region, and the red area indicates the danger region.

When the vehicle is within the safe region, over time, the tire slip angles will gradually converge to the stable equilibrium point, and the vehicle will be in a stable state, possessing good stability. At this point, the focus is on enhancing the maneuverability. When the vehicle is in the danger region, as time progresses, without the intervention of other factors, the front and rear tire slip angles will gradually diverge, placing the vehicle in an unstable state, where the focus shifts to enhancing stability. The handling stability index,  $\xi_{2}$ , is defined as follows:

$$
\xi_ {2} = \frac {R ^ {\prime}}{R _ {3}} \tag {20}
$$

Specifically, when  $0 < \xi_{2} \leq 1$ , the vehicle is stable and tire slip angles will converge over time. The focus is on maneuverability. When  $1 < \xi_{2}$ , the vehicle enters the danger region, where slip angles diverge, leading to instability. The focus is on stability improvement.

Both  $\xi_{1}$  and  $\xi_{2}$  are key parameters for characterizing handling stability. To further represent the stability margin, as shown in Fig. 10, a fuzzy logic scheme is used to fuse  $\xi_{1}$  and  $\xi_{2}$  into a single stability index  $\xi$ . Specifically, the two handling stability indices,  $\xi_{1}$  and  $\xi_{2}$ , are selected as input variables to the fuzzy inference system, while the overall vehicle stability margin  $\xi$  is defined as the output variable. The universe of discourse for  $\xi_{1}$  is set to [0, 1.5] and is partitioned into six fuzzy subsets: ZO (Zero), PS (Positive Small), PM (Positive Medium), PB (Positive Big), PL (Positive Large), and PBL (Positive Very Large). The sec

ond input variable,  $\xi_{2}$ , is defined over the domain [0, 1] and is divided into five fuzzy subsets: ZO, PS, PM, PB, and PL. The output variable  $\xi$ , representing the final stability margin, also has a universe of discourse [0, 1] and is categorized into five fuzzy subsets identical to those of  $\xi_{2}$ : ZO, PS, PM, PB, and PL. All membership functions are defined using Gaussian shapes to ensure smoothness and continuity. The complete fuzzy rule base is summarized in Table 2. This stability index  $\xi$  quantifies the stability margin under various states and provides a foundation for the tracking controller design.

# 3 Controller design

# 3.1 The underlying MPC model

The predictive model is based on the 3-DOF vehicle dynamics model, the state variable is  $x = \left[V_y, V_x, \varphi, \dot{\varphi}, X, Y\right]$ , the control variable is  $u = \left[a_x, \delta_f\right]^T$ .

The nonlinear system is described by:

$$
\dot {x} = f (x, u) \tag {21}
$$

The nonlinear model is approximated by a first-order Taylor expansion around a reference point  $(x_r, u_r)$ :

$$
\dot {x} = f \left(x _ {r}, u _ {r}\right) + A _ {1} \left(x - x _ {r}\right) + B _ {1} \left(u - u _ {r}\right) + e _ {t} \tag {22}
$$

where  $e_t$  represents the linearization errors, and

Table 2 Fuzzy logic design for stability margin  

<table><tr><td>ξ1</td><td colspan="5">ξ2</td></tr><tr><td></td><td>ZO</td><td>PS</td><td>PM</td><td>PB</td><td>PL</td></tr><tr><td>ZO</td><td>ZO</td><td>ZO</td><td>ZO</td><td>PS</td><td>PS</td></tr><tr><td>PS</td><td>ZO</td><td>PS</td><td>PS</td><td>PM</td><td>PM</td></tr><tr><td>PM</td><td>PS</td><td>PM</td><td>PM</td><td>PB</td><td>PB</td></tr><tr><td>PB</td><td>PM</td><td>PM</td><td>PB</td><td>PB</td><td>PL</td></tr><tr><td>PL</td><td>PB</td><td>PB</td><td>PB</td><td>PL</td><td>PL</td></tr><tr><td>PBL</td><td>PL</td><td>PL</td><td>PL</td><td>PL</td><td>PL</td></tr></table>

where  $\tilde{A}_k = I + TA_1$ ,  $\tilde{B}_k = TB_1$ ,  $I$  is identity matrix.

The MPC cost function ensures trajectory tracking and stability and is formulated:

$$
J = \sum_ {i = 1} ^ {N p} \left\| \eta_ {k + i} - \eta_ {r | k + i} \right\| _ {Q} ^ {2} + \sum_ {i = 1} ^ {N c} \| \Delta u _ {k + i} \| _ {R} ^ {2} + \rho \varepsilon^ {2} \tag {24}
$$

where  $\eta_{k + i}$  and  $\eta_{r|k + i}$  are the system state vector and reference state vector at time step  $k + i$ ,  $Q$  is the state weighting matrix,  $R$  is the control weighting matrix,  $\rho$  is the regularization parameter to balance the slack variable penalty, and  $\varepsilon$  is the slack variable.

The objective function is reformulated as a quadratic programming problem as follows:

$$
A _ {1} = \left[ \begin{array}{c c c c} \frac {C _ {f} ^ {t} + C _ {r} ^ {t}}{m V _ {x}} & \frac {\partial f _ {V _ {y}}}{\partial V _ {x}} & 0 & \frac {a C _ {f} ^ {t} - b C _ {r} ^ {t}}{m V _ {x}} - V _ {x} 0 0 \\ \dot {\varphi} & 0 & 0 & V _ {y} 0 0 \\ 0 & 0 & 0 & 1 0 0 \\ \frac {a C _ {f} ^ {t} - b C _ {r} ^ {t}}{I _ {z} V _ {x}} & \frac {\partial f _ {\dot {\varphi}}}{\partial V _ {x}} & 0 & \frac {a ^ {2} C _ {f} ^ {t} + b ^ {2} C _ {r} ^ {t}}{I _ {z} V _ {x}} 0 0 \\ \cos \varphi & \sin \varphi & V _ {x} \cos \varphi - V _ {y} \sin \varphi & 0 0 \\ - \sin \varphi & \cos \varphi & - V _ {x} \sin \varphi - V _ {y} \cos \varphi & 0 0 \end{array} \right], B _ {1} = \left[ \begin{array}{c c c c} 0 & 1 0 & 0 & 0 0 \\ - \frac {C _ {f} ^ {t}}{m} & 0 0 & - \frac {a C _ {f} ^ {t}}{I _ {z}} & 0 0 \end{array} \right] ^ {T}.
$$

The sampling period  $T$  is set to  $10\mathrm{ms}$ , the results of approximate discretization and exact discretization are virtually identical, therefore Eq. 22 is discretized using the first-order Euler (forward-difference) method [41], the system is defined as:

$$
x _ {k + 1} = \tilde {A} _ {k} x _ {k} + \tilde {B} _ {k} u _ {k} + e _ {k} \tag {23}
$$

$$
J = \frac {1}{2} x ^ {T} H x + f ^ {T} x \mathrm {s . t} \left\{ \begin{array}{c} A x \leq b \\ l b \leq x \leq u b \end{array} \right. \tag {25}
$$

where  $H$  represents the Hessian matrix and  $f$  is linear cost term.

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/118da3ac91d2c7432cb9f4494540f45b2d07923fcbf4102db9fa815e87525f07.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/992e330a3e3059bf977d3b9c06d156412e6934307cd2de16dc9cc4b9ce356597.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/0d584a21bf2133ff9bfa0f221440f0681ea9978217da62446020a22d48c3f2b0.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/4bd727fc4d80b06074e8dcde61320fb9c7fe5d8cb4d0e68116dea64cd038733c.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/1f0615ef2799fc0396843c201c868667faaad6a5713af38f5f95b188018f8f82.jpg)  
Fig. 11 Influence of weight  $q_{Y}$  under low adhesion road

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/af3ae1da6f8ca5d1c77bf5b707876c38c357518e2419dfd62d1cc7934dfc7b6b.jpg)

The constraints on the front wheel steering angle and acceleration are defined as follows:

$$
\left\{ \begin{array}{l} \delta_ {f, \min } \leq \delta_ {f} \leq \delta_ {f, \max } \\ \Delta \delta_ {f, \min } \leq \Delta \delta_ {f} \leq \Delta \delta_ {f, \max } \\ - \mu g \leq a _ {x} \leq \mu g \\ \Delta a _ {x, \min } \leq \Delta a _ {x} \leq \Delta a _ {x, \max } \end{array} \right. \tag {26}
$$

The resulting convex QP is solved using a CVXGEN-generated solver, which employs a highly optimized primal-dual interior-point method (see Appendix A for a detailed sensitivity analysis and comparison across different QP solvers). This solver is well-suited for MPC-based QP formulations due to its low overhead, numerical stability, and fast convergence.

# 3.2 Sensitivity analysis under different weight

We further analyze the impact of the lateral position weight  $q_{Y}$  and longitudinal speed weight  $q_{Vx}$  on the controller.

For  $q_{Y}$  under low friction conditions  $(\mu = 0.2)$  and a target speed of  $45\mathrm{km / h}$ ,  $q_{Y}$  is set to 700, 1000, 3000, and 5000, respectively. As shown in Fig. 11, different values of  $q_{Y}$  have a significant impact on control performance. When  $q_{Y} = 1000$ , the peak values of yaw rate

and sideslip angle are small, with no noticeable oscillations. The front wheel sideslip angle remains within a small range, ensuring good stability. The front wheel steering angle also changes smoothly. However, in this case, the lateral position error is relatively large. As  $q_{Y}$  increases, the lateral position error decreases. A higher  $q_{Y}$  causes the vehicle to frequently adjust the front wheel steering angle. During the lane departure phase of the double lane-change maneuver, the vehicle tends to generate larger front wheel steering angles to quickly align with the desired trajectory. This increases the tire slip angle range beyond the stable region, leading to degraded yaw rate and sideslip angle responses, ultimately reducing stability. When  $q_{Y}$  decreases, the lateral position error increases. The larger lateral error results in greater front wheel steering angles, which in turn expand the tire slip angle range. This leads to poor yaw rate and sideslip angle responses, further compromising stability.

For  $q_{Y}$  under high friction conditions  $(\mu = 0.8)$  and a target speed of  $60\mathrm{km / h}$ ,  $q_{Y}$  is set to 700, 1000, 3000, and 5000, respectively. As shown in Fig. 12, under different  $q_{Y}$  values, both the yaw rate and sideslip angle remain within a small range, with smooth variations and no significant fluctuations. The tire slip angle also stays within the linear region. When  $q_{Y} = 5000$ , the control performance is optimal, with minimal lateral trajectory

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/3bcdffca0a68ff28d2ac078372a64007cf606b62af632c1d0beb7af37577203c.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/97df4b0da55ae2e6b686be467abe7bf4ebc672eb10a4dd058c515d6642c74239.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/8a08c373e5dfb8171ab05b9df6d2a8566fdb6eaad84cc1df10b0b36e57b0ec26.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/70a277def19e51fdeae9e2510b645d1c49498746a9ba80ff0eb11190b4cbd006.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/92dc3bc553e646e919b04faad2d77872204aa4d3da039bee0aa28337304cd709.jpg)  
Fig. 12 Influence of weight  $q_{Y}$  under high adhesion road

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/507fbd18e23d927ae585aa4e05c5ddc884b091f9ba6a6b448956cf5c44ee1699.jpg)

tracking error while other state variables remain at low levels.

For  $q_{Vx}$  under low friction conditions  $(\mu = 0.2)$  and a target speed of  $45\mathrm{km / h}$ ,  $q_{Vx}$  is set to 100, 500, 700, and 900, respectively. As shown in Fig. 13, under the condition,  $q_{Vx}$  has a significant impact on the performance of the trajectory tracking controller. When  $q_{Vx} = 100$ , the lateral position error is small, and the yaw rate and sideslip angle responses are stable with no noticeable oscillations. The front tire slip angle remains within a small range, ensuring good stability, and the front wheel steering angle changes smoothly. However, in this case, the longitudinal speed tracking error is the largest. As  $q_{Vx}$  increases, the longitudinal speed tracking accuracy improves, but both stability and lateral position tracking accuracy deteriorate significantly. On a low friction road, the adhesion limit is greatly reduced. A higher  $q_{Vx}$  means a stronger emphasis on the longitudinal speed tracking task, leaving insufficient lateral tire force to follow the reference lateral position. This results in a significant increase in lateral position error and a larger front wheel steering angle control effort. Consequently, the front tire slip angle range expands, the yaw rate and sideslip angle responses worsen, and stability deteriorates.

For  $q_{Vx}$  under high friction conditions  $(\mu = 0.8)$  and a target speed of  $60~\mathrm{km / h}$ ,  $q_{Vx}$  is set to 100,

500, 700, and 900, respectively. As shown in Fig. 14, when  $q_{Vx} = 900$ , the control performance is optimal, achieving the highest longitudinal speed tracking accuracy while maintaining stability and lateral position tracking accuracy without significant changes. As  $q_{Vx}$  decreases, the longitudinal speed tracking error increases, reaching its maximum when  $q_{Vx} = 100$ . At this point, the vehicle is on a high friction road, where the adhesion limit is significantly higher. As shown in Fig. 14(f), when achieving good lateral position tracking, the tire slip angle remains within a small range, indicating that there is still sufficient remaining tire force. In this case, increasing the longitudinal speed weight  $q_{Vx}$  significantly enhances the longitudinal speed tracking task without affecting stability or lateral position tracking. Conversely, using a smaller  $q_{Vx}$  reduces the potential for accurate longitudinal speed tracking.

# 3.3 Weight adaptive via the stability index

Unlike the conventional MPC controller with fixed weighting coefficients, we establish a relationship between the stability index and the MPC weighting coefficients to realize an adaptive MPC, which dynamically adjusts its weights to balance stability and tra

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/304fdbca602381c4a45b7288c938a7ac4194f9ca76cfcc7a4a9115ca51afc2a9.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/b5f0d96cc001b30fddc00e8a6108e53e4e02ee313052da7a7c9a5573fb33d6a1.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/310904757d0eb95d72c60557c0702c27b63fecb16aa05a3f316df392a2a1e162.jpg)  
Fig. 13 Influence of weight  $q_{Vx}$  under low adhesion road

j trajectory tracking. Specifically, on a low adhesion road, reducing the lateral position weighting  $q_{Y}$  helps maintain stability, while on a high adhesion road, increasing  $q_{Y}$  boosts lateral position tracking accuracy without compromising stability. Likewise, increasing the longitudinal speed weighting  $q_{V_x}$  on a low adhesion road can undermine stability, whereas on a high adhesion road it enhances overall controller performance. Based on the above considerations, we formulate a weight self-adjustment strategy using the phase plane-based stability index, as illustrated in Fig. 15.

# 4 Experiment and discussion

# 4.1 Hardware-in-the-Loop experiments

The real-time and effective performance of the proposed control strategy is verified via hardware-in-the-loop (HIL). As shown in Fig. 16, the HIL platform is constructed based on Veristand RT and CarSim RT systems and consists of the following five main parts: a host computer (VeriStand RT and CarSim RT UI), a target machine (running Veristand RT and CarSim RT systems), a driving simulator, a controller and PC (algorithm deployment, monitoring and calibration), and the ZLG USB-CAN interface. The subsystems are

connected via CAN or UDP. A standard C-Class vehicle in CarSim is used as the target vehicle. The main parameters are presented in Table 3. The simulation is conducted in a MATLAB-CarSim co-simulation environment. The control period is set to  $10\mathrm{ms}$  with a prediction horizon of  $N_{p} = 15$  and a control horizon of  $N_{c} = 10$ . The control architecture is illustrated in Fig. 17. The tire slip angles and friction coefficient are estimated using the method from our previous work [42]. The lower-level actuator algorithm converts the desired longitudinal acceleration from the tracking controller into engine torque and brake master cylinder pressure. A PID controller [43] monitors the difference between the desired acceleration and the actual acceleration, then adjusts the engine output or brake force accordingly. This design ensures prompt and accurate control responses, enabling the vehicle to follow the commanded longitudinal acceleration under a wide range of operating conditions. Moreover, the double lane change (DLC) maneuver is conducted on both high and low adhesion roads to verify the effectiveness of the proposed controller. In the scenario, the fixed-weight MPC [44] and AMPC controllers are compared. All controllers are evaluated under the same vehicle dynamics model, initial conditions, road friction levels, reference trajectories, and disturbance settings. In the fixed-weight MPC con

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/cde58a0deda8c61bdef8a66f049d06bc12497c994502ef52a969dbbb64920f5a.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/53bfb08e042c072721ca10be76c31257b37a4403619b5a46e6bfabaf8c4a1a74.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/95d437b7eee2e521e622bb5afc04f17348aee313eadcfe4d5172ab3dcf407c15.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/95859c427784c6f7a3b355b1c7ef630b129601977e6cf64bf4f0d288f1df7125.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/efc2914feb37c9c20b799a972e0bc40f59d6c061cba5ac979f843dbd7e5409dc.jpg)  
Fig. 14 Influence of weight  $q_{Vx}$  under high adhesion road

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/9bd2cd193cd802f53fdf93b32c1487660525ebf33b9f54c595bf0620134c52c5.jpg)

troller, the selection of weights is guided by the sensitivity analysis under different weight configurations presented in Section 3.2, along with the performance metrics used in Section 4.2. As shown in Fig. 18, the results illustrate how different weighting choices affect trajectory tracking accuracy and vehicle stability. When  $q_{Y} = 5000$ , the controller achieves the best overall performance: it yields the smallest maximum lateral tracking error while maintaining other performance metrics within acceptable bounds. Notably, although the maximum yaw rate RMSE reaches 16 deg/s under high-friction conditions, this value is still well below the yaw rate stability boundary defined by  $\mu g / V_x$ , which is approximately 27 deg/s. This confirms that the vehicle remains stable. Similarly, when  $q_{V_x} = 100$ , the controller achieves the best trade-off between longitudinal tracking accuracy and lateral stability. Based on these results, we selected  $q_{Y} = 5000$  and  $q_{V_x} = 100$  for the fixed-weight MPC controller in our comparative experiments. Moreover, the cost function structure is identical to the proposed AMPC, but without online weight adjustment. The same prediction model and constraints were used to ensure a fair baseline.

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/fbbe58304b0e31f3bc5e71db814e848bc8e8fb53a0084b6efaae576ba4a95c1a.jpg)  
Fig. 15 Weight adaptive based on phase plane stability index

# 4.2 Metrics

To evaluate the performance of the proposed trajectory tracking controller, the root-mean-square error (RMSE) metrics are employed, as shown in Table 4. Specifically,  $\mathrm{RMSE\_Y}$  measures how accurately the vehicle follows the lateral reference  $\mathrm{RMSE\_Vx}$  quantifies speed-tracking accuracy relative to  $V_{x,ref}$ , and  $\mathrm{RMSE\_var}$  and  $\mathrm{RMSE\_\dot{\varphi}}$  capture yaw angle and yaw rate tracking precision, respectively. Lastly,  $\mathrm{RMSE\_}\beta$

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/8f4e0e0398e59338784c0fb252845470ee55c4f9c9ac2c08595a0a5ece974bc5.jpg)  
Fig. 16 The hardware-in-the-loop platform

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/8ef16404339ba477723846b75253a263fa4528d15e155e36d902ecc8a88c59c8.jpg)  
Fig. 17 Schematic diagram via AMPC

Table 3 Main Parameters  

<table><tr><td>Parameter</td><td>Description</td><td>Value (units)</td></tr><tr><td>m</td><td>vehicle mass</td><td>1412 kg</td></tr><tr><td>Iz</td><td>moment of inertia</td><td>1536.7 kg·m2</td></tr><tr><td>a</td><td>distance from CoG to front axle</td><td>1050 mm</td></tr><tr><td>b</td><td>distance from CoG to rear axle</td><td>1860 mm</td></tr><tr><td>qY,min-max</td><td>adaptive lateral position weight</td><td>1000-5000</td></tr><tr><td>qVx,min-max</td><td>adaptive longitudinal speed weight</td><td>100-900</td></tr><tr><td>MPC controller qY</td><td>fixed lateral position weight</td><td>5000</td></tr><tr><td>MPC controller qVx</td><td>fixed longitudinal speed weight</td><td>100</td></tr><tr><td>Q</td><td>control weighting</td><td>diag(0.1,10)</td></tr><tr><td>ρ</td><td>slack-penalty</td><td>103</td></tr></table>

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/219770e404dbed94630391f0e8d912cf57c55afe250d2e4b082e226f4e2fce1e.jpg)  
(a)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/7dd4f59cf2aef5a87b7f55fb78becface7cc5be39c46cd3cea19615f52054836.jpg)  
(b)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/4373ce4abf610def4edf87ee53c945de49ebfd210a1972205efd0733a2b7d25b.jpg)  
(c)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/6c551fc7cfdb9a5ece2b24bd91470ab787456175fd3e216981a9e969be23ebc2.jpg)  
(d)  
Fig. 18 RMSE comparison under different control weight settings. (a-b) Sensitivity to  $q_{Y}$  under low and high friction roads. (c-d) Sensitivity to  $q_{V_x}$  under low and high friction roads

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/84a1385d7683ca0b2d35881184af234860aac80129fd29bf404c912bb42d3bee.jpg)  
Fig. 19 Trajectory tracking performance on variable speed high adhesion DLC maneuver. (a) Lateral position. (b) Lateral position error. (c) Longitudinal speed. (d) Yaw angle

evaluates stability by indicating how far the sideslip angle  $\beta$  deviates from zero on average. A lower value for each metric corresponds to improved performance, whether in lateral positioning, speed tracking, yaw tracking, or overall stability.

# 4.3 Variable speed high adhesion DLC maneuver

Variable speed high adhesion DLC maneuver can examine the controller's adaptability to speed variations and robustness. As shown in Fig. 19(a-b), both trajectory tracking controllers provide comparable lateral position tracking accuracy. In the MPC controller, the lateral position weight is assigned its maximum value,  $q_{Y} = q_{Y,max}$ , while the AMPC controller uses adaptive weighting to achieve equally effective lateral tracking. As shown in Fig. 19(c), the AMPC controller significantly improves longitudinal speed tracking accuracy. The MPC controller's maximum longitudinal speed tracking error is  $5.28\mathrm{km / h}$  whereas the AMPC controller reduces this error to  $1.95\mathrm{km / h}$  representing a  $63.07\%$  decrease in the maximum longitudinal speed error.

As shown in Figs. 20(a-c), both trajectory tracking controllers demonstrate robust stability. The yaw rate closely follows the reference, the sideslip angle remains small, and the tire slip angles stay within their linear

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/0ab0b2a97a60b9afb01773f4da5ee62b6464f506d65725bef85f29c3dabb81d5.jpg)  
Fig. 20 Stability performance on variable speed high adhesion DLC maneuver. (a) Yaw rate. (b) Sideslip angle. (c)  $\alpha_{f} - \alpha_{r}$  phase plane. (d) Tire force usage

range. The AMPC controller exhibits slightly improved stability compared to the MPC controller.

Furthermore, tire force usage evaluates whether the vehicle is approaching the tire grip limit. This metric is defined as the resultant of lateral and longitudinal tire forces divided by the product of the road friction coefficient and the tire normal load, expressed as:  $\sqrt{F_x^2 + F_y^2} / (\mu F_z)$ , where  $F_{x}$  and  $F_{y}$  represent the longitudinal and lateral tire forces, and  $F_{z}$  is the normal tire load. Fig. 20(d) illustrates tire force usage, compared with the MPC controller, the AMPC controller maintains lower overall tire usage. This indicates that the AMPC method operates farther from the grip limit, reducing the likelihood of excessive slip and thereby enhancing overall handling stability.

As shown in Fig. 21(a), the stability index  $\xi$  remains below 0.25, indicating good stability. Consequently, the primary focus is to enhance lateral position and longitudinal speed tracking. This is reflected in the control weight adjustments, where the lateral position weight  $q_{Y}$  and the longitudinal speed weight  $q_{V_x}$  are both increased. This is consistent with the experimental results shown in Figs. 21(b-c).

Table 4 Evaluation metrics  

<table><tr><td>Description</td><td>Equation</td></tr><tr><td>Lateral position tracking metric</td><td>RMSE_ Y = √(Σi=1N(Y-Yref)2/N</td></tr><tr><td>Speed tracking metric</td><td>RMSE_ Vx = √(Σi=1N(Vx-Vxref)2/N</td></tr><tr><td>Yaw angle tracking metric</td><td>RMSE_ φ = √(Σi=1N(φ-φref)2/N</td></tr><tr><td>Yaw rate tracking metric</td><td>RMSE_ ð = √(Σi=1N(ð-ðref)2/N</td></tr><tr><td>Stability metric</td><td>RMSE_ β = √(Σi=1N(β)2/N</td></tr></table>

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/098115bc2526e54c0e42741455cf209a5fbabecf44aa252f4c3f5cce4a403bfa.jpg)  
Fig. 21 Stability margin and weight update on variable speed high adhesion DLC maneuver

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/2018c3cfee004c0f1ae76faa6d121a508b3c442fece4f389e761c4c357f70909.jpg)  
Fig. 22 Metrics performance on variable speed high adhesion DLC maneuver

As shown in Fig. 22, the AMPC controller provides comparable lateral position tracking performance to the MPC controller. However, it significantly improves longitudinal speed tracking, reducing the RMSE from 1.49 to 0.52, a  $65.10\%$  decrease. These findings indicate that our method substantially enhances overall trajectory tracking while maintaining stability.

# 4.4 Split friction DLC maneuver

During the transition maneuver, the vehicle travels at  $V_{x} = 45 \mathrm{~km/h}$  on a high friction road ( $\mu = 0.8$ ). Once the vehicle reaches  $100 \mathrm{~m}$ , the friction coefficient drops to  $\mu = 0.2$ . Split friction DLC maneuver produces a sharp transition from high to low friction conditions, this maneuver tests the controller's robustness and real-time responsiveness when road conditions degrade quickly. In order to improve stability under these conditions, the AMPC controller reduces the lateral position tracking weight based on the stability index, thereby sacrificing some lateral position tracking accuracy.

As shown in Fig. 23, compared with the MPC controller, the AMPC controller exhibits significantly higher longitudinal speed tracking accuracy. On the high friction section, the MPC controller's maximum longitudinal speed control error is  $5.80\mathrm{km / h}$ , whereas the AMPC controller reduces this error to  $2.11\mathrm{km / h}$ , a  $63.62\%$  decrease.

Figs. 24(a-b) show that the AMPC controller substantially improves the yaw rate and sideslip angle responses: the peak yaw rate decreases from 10.41 deg/s to 1.96 deg/s (81.17% reduction), and the peak sideslip angle decreases from 1.48 deg to 0.38 deg

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/d5703d60aa6690594ecf40b515fe4ed567e616350ad5d1e322b27b2049035de7.jpg)  
Fig. 23 Trajectory tracking performance on split friction DLC maneuver. (a) Lateral position. (b) Lateral position error. (c) Longitudinal speed. (d) Yaw angle

(74.32% reduction). As depicted in Fig. 24(c), the AMPC controller constrains the tire slip angles to a much narrower range under low friction conditions, thereby enhancing stability. In Fig. 24(d), once the vehicle has traveled approximately  $150\mathrm{m}$  the front wheel slip angle under MPC far exceeds the stable boundary. In contrast, the AMPC controller, by adjusting the lateral position tracking weight, maintains the slip angle near the stable boundary, further improving stability. When  $\mu$  suddenly drops from 0.8 to 0.2 at the  $100\mathrm{m}$  the vehicle is subjected to an external disturbance (a sudden change in road excitation). Fig. 24(e) shows that, with the fixed-weight MPC, the front-wheel lateral force utilisation spikes to 2.44, reaching saturation. Whereas the AMPC, by promptly reducing  $q_{Y}$  keeps that utilisation below 0.87. Simultaneously, the stability index  $\xi$  leaps from 0.22 to 0.63, triggering a downward adjustment of the weights (Fig. 25(b)). This response brings the vehicle back into the safe region within 2 s.

In Fig. 25(a), under high friction conditions, the stability index  $\xi$  remains below 0.25, indicating good stability. At this point, the main objective is to enhance lateral position tracking accuracy and longitudinal speed tracking accuracy, provided that stability is maintained. This is reflected by increasing both the lateral position weight  $q_{Y}$  and the longitudinal speed weight  $q_{V_x}$ . When the vehicle enters the low friction road, over

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/240eda092eba4ab2952917b5771d70a3fbbc850ea158f44fed9ae5856ecaf2bb.jpg)  
Fig. 24 Stability performance on split friction DLC maneuver. (a) Yaw rate. (b) Sideslip angle. (c)  $\alpha_{f} - \alpha_{r}$  phase plane. (d) Front tire slip angle limit. (e) Tire force usage

all stability deteriorates, so the chief priority becomes ensuring stability. In this condition, the longitudinal speed weight  $q_{V_x}$  is not adjusted and remains fixed at  $q_{V_x, \min}$ . Based on the stability index, the AMPC controller dynamically lowers the lateral position weight  $q_{Y}$ . These observations are consistent with the results in Figs 25(b-c).

As shown in Fig. 26, the AMPC controller's lateral position RMSE is higher than the MPC controller's, because the lateral position error increases on low friction roads. Nevertheless, the controller's remaining performance indicators are all improved: the longitudinal speed RMSE decreases from 1.60 to 1.45 (9.38% improvement), the yaw angle RMSE from 0.75 to 0.62 (17.33% improvement), the yaw rate RMSE from 2.12 to 1.40 (33.96% improvement), and the sideslip angle RMSE from 0.25 to 0.19 (24.00% improvement). Hence, under rapidly changing road conditions, the adaptive controller markedly improves stability and overall tracking performance at the cost

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/e6eab9948ea6565878c8a8fc15308709ef399e03006e8e544959689aaa86939b.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/0325c4794f7e5744249814b38800120c50813a6e6650e84310290543e8e2a1f7.jpg)  
Fig. 27 Computation time on split friction DLC maneuver

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/9cf02bcbe7af63b657651ac8d188abf35c73c6a7a6ffc4321716bc9e736bcfff.jpg)  
Fig. 25 Stability margin and weight update on split friction DLC maneuver

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/09b2652437c9a92bf4e00a742bd8d61d111952ab8862b83754587e67ee541f11.jpg)  
Fig. 26 Metrics performance on split friction DLC maneuver

of a small increase in lateral error, a trade-off that is often acceptable for safety. Overall, when subjected to strong disturbances, the AMPC deliberately sacrifices a degree of lateral tracking accuracy in exchange for an increased stability margin, exactly the design intent of the "stability-first" strategy.

Fig. 27 shows the computation time. The solver consistently operates below  $2\mathrm{ms}$ , thereby providing a significant time margin relative to the  $10\mathrm{ms}$  real-time requirement. The computation time indicates that the proposed approach efficiently balances model complexity with real-time performance, confirming its feasibility for in-vehicle implementation.

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/019c145412e5f47f038b7629c2f879979b1bccd3a8bb6d0c899f04f7c91d426e.jpg)

# 5 Conclusion

This work introduces a stability margin approach for trajectory tracking control under low adhesion conditions. By defining the front and rear tire slip angles in the  $\alpha_{f} - \alpha_{r}$  phase plane and locating saddle points, the proposed method captures critical stability boundaries that reflect the extent to which tire forces are approaching saturation. This framework enables a model predictive controller to adaptively balance tracking performance and stability via a tailored weighting strategy.

Experimental validation through hardware-in-the-loop tests showed that the proposed strategy is effective in scenarios such as double lane changes on variable speed and split friction transitions. Compared with the conventional MPC strategy, the adaptive MPC reduces the longitudinal RMSE by  $65.10\%$  and  $9.38\%$  on variable speed double lane change maneuver and split friction double lane change maneuver, the RMSE of yaw rate and sideslip angle decreased by  $33.96\%$  and  $24.00\%$ , respectively. The phase-plane saddle-point fits depend on the specific vehicle and tire model, and variations in the road friction coefficient can induce transient oscillations in both the stability index and control weights, thereby losing tracking accuracy. Future work will investigate online parameter adaptation and learning-based modules to improve robustness and extend applicability to more complex scenarios.

As part of our ongoing work, we are developing online parameter adaptation stability boundary modules to enhance robustness and generalizability across time-varying vehicle parameters and road conditions. A preliminary exploration of this direction is provided in [23].

Author contributions Ziyu Song: Conceptualization, Methodology, Software, Validation, Writing- review & editing, Writing - original draft preparation, Funding acquisition. Junlong Wang: Writing - review & editing, Software, Validation, Data curation, Conceptualization. Yang Liang: Writing - original draft preparation, Writing - review & editing, Validation, Data curation, Conceptualization. Zhiqi Mao: Writing - original draft preparation,

Writing - review & editing, Validation, Data curation, Conceptualization. Haitao Ding: Writing - review & editing, Conceptualization, Funding acquisition, Supervision.

Funding This work was supported in part by the China Automobile Industry Innovation and Development Joint Fund under Grant No. U1864206, and in part by the Science and Technology Major Project of Jilin Province under Grant No. 20220301033GX.

Data Availability The data that support the findings of this study are available from the corresponding author upon reasonable request.

# Declarations

Competing interests The author(s) declared no potential conflicts of interest with respect to the research, authorship, and/or publication of this article.

# Appendix A Sensitivity to different QP solvers

This section evaluates the controller's sensitivity to different QP solvers and offers guidance for solver selection. We compare four solvers: quadprog (a general-purpose interior-point method), OSQP (an ADMM-based operator-splitting solver), qpOASES (an online active-set method), and CVXGEN (a primal-dual interior-point solver) on a lane-change maneuver at prediction horizons  $N_{p} = 15$  and 30, each with warm (w.) and without warm starts (w.o.). As shown in Figs. 28(a-b), all solvers deliver virtually identical lateral and yaw tracking accuracy. Fig. 28(c) highlights their differing computation times: CVXGEN achieves sub-milisecond average latency with minimal jitter, while the other solvers incur modestly higher runtimes. Warm starts consistently reduce solve times by 20-50%. These findings confirm that solver choice, prediction horizon length, and warm-start strategy are the primary sensitivity factors in meeting the 10 ms real-time requirement. Accordingly, we recommend CVXGEN for practical implementations.

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/85f00f88c99dc22199f3208d179f12fe96cf9d1ef338d83084fc1964f1fe06ab.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/082b06437f2503036faeab48e67672946f62df18d827fc5dd0e83bc9757f6571.jpg)

![](https://cdn-mineru.openxlab.org.cn/result/2025-12-29/f01f4b66-5c59-435e-9444-e5c02a19fff9/c601e2691e1c3a2246b25997ec29d43dbaaea0a2d548ce36374a2cd67f707ba6.jpg)  
Fig. 28 Comparison of using different solvers. (a) Lateral position. (b) Yaw angle tracking performance. (c) Computation time

# References

1. Stano, P., Montanaro, U., Tavernini, D., Tufo, M., Fiengo, G., Novella, L., Sorniotti, A.: Model predictive path tracking control for automated road vehicles: A review. Annu. Rev. Control 55, 194-236 (2023). https://doi.org/10.1016/j.arcontrol.2022.11.001  
2. Wang, Z., Liang, Z., Ding, Z.: Observer-based prescribed performance path-following control for autonomous ground vehicles via error shifting method. Nonlinear Dyn. 112(12), 10061-10080 (2024). https://doi.org/10.1007/s11071-024-09591-y  
3. Zheng, H., Li, Y., Zheng, L., Hashemi, E.: Koopman-based hybrid modeling and zonotopic tube robust mpc for motion control of automated vehicles. IEEE Trans. Intell. Transp. Syst. 25(10), 13598-13612 (2024). https://doi.org/10.1109/TITS.2024.3403656  
4. Joa, E., Hyun, Y., Park, K., Kim, J., Yi, K.: Model predictive control based stability control of autonomous vehicles on low friction road. In: IEEE Intelligent Vehicles Symposium (IV), pp. 419-424 (2020). https://doi.org/10.1109/IV47402.2020.9304622

5. Rokonuzzaman, M., Mohajer, N., Nahavandi, S., Mohamed, S.: Model predictive control with learned vehicle dynamics for autonomous vehicle path tracking. IEEE Access 9, 128233-128249 (2021). https://doi.org/10.1109/ACCESS.2021.3112560  
6. Chu, D., Li, H., Zhao, C., Zhou, T.: Trajectory tracking of autonomous vehicle based on model predictive control with pid feedback. IEEE Trans. Intell. Transp. Syst. 24(2), 2239-2250 (2022). https://doi.org/10.1109/TITS.2022.3150365  
7. Zhai, L., Wang, C., Hou, Y., Liu, C.: Mpc-based integrated control of trajectory tracking and handling stability for intelligent driving vehicle driven by four hub motor. IEEE Trans. Veh. Technol. 71(3), 2668-2680 (2022). https://doi.org/10.1109/TVT.2022.3140240  
8. Zhou, Z., Rother, C., Chen, J.: Event-triggered model predictive control for autonomous vehicle path tracking: Validation using carla simulator. IEEE Trans. Intell. Veh. 8(6), 3547-3555 (2023). https://doi.org/10.1109/TIV.2023.3266941  
9. Chen, G., Zhao, X., Gao, Z., Hua, M.: Dynamic drifting control for general path tracking of autonomous vehicles. IEEE Trans. Intell. Veh. 8(3), 2527-2537 (2023). https://doi.org/10.1109/TIV.2023.3235007  
10. Meng, Q., Qian, C., Chen, K., Sun, Z.Y., Liu, R., Kang, Z.: Variable step mpc trajectory tracking control method for intelligent vehicle. Nonlinear Dyn. 112(21), 19223-19241 (2024). https://doi.org/10.1007/s11071-024-10042-x  
11. Yan, Y., Yu, S., Gao, X., Wu, D., Li, T.: Continuous and periodic event-triggered sliding-mode control for path following of underactuated surface vehicles. IEEE Trans. Cybern. 54(1), 449-461 (2024). https://doi.org/10.1109/TCYB.2023.3265039  
12. Ding, C., Ding, S., Wei, X., Mei, K.: Output feedback sliding mode control for path-tracking of autonomous agricultural vehicles. Nonlinear Dyn. 110(3), 2429-2445 (2022). https://doi.org/10.1007/s11071-022-07739-2  
13. Hu, Y., Fu, J., Wen, G.: Safe reinforcement learning for model-reference trajectory tracking of uncertain autonomous vehicles with model-based acceleration. IEEE Trans. Intell. Veh. 8(3), 2332-2344 (2023). https://doi.org/10.1109/TIV.2022.3233592  
14. Chai, R., Niu, H., Carrasco, J., Arvin, F., Yin, H., Lennox, B.: Design and experimental validation of deep reinforcement learning-based fast trajectory planning and control for mobile robot in unknown environment. IEEE Trans. Neural Networks Learn. Syst. 35(4), 5778-5792 (2024). https://doi.org/10.1109/TNNLS.2022.3209154  
15. Ghandriz, T., Jacobson, B., Nilsson, P., Laine, L.: Trajectory-following and off-tracking minimisation of long combination vehicles: A comparison between nonlinear and linear model predictive control. Veh. Syst. Dyn. 62(2), 277-310 (2024). https://doi.org/10.1080/00423114.2022.2164513  
16. Xu, F., Zhang, X., Chen, H., Hu, Y., Wang, P., Qu, T.: Parallel nonlinear model predictive controller for real-time path tracking of autonomous vehicle. IEEE Trans. Ind. Electron. 71(12), 16503-16513 (2024). https://doi.org/10.1109/TIE.2024.3390738  
17. Mercorelli, P.: Trajectory tracking using mpc and a velocity observer for flat actuator systems in automotive applications. In: 2008 IEEE International Symposium on Industrial Electronics, pp. 1138-1143 (2008). https://doi.org/10.1109/ISIE.2008.4677259

18. Mercorelli, P.: Combining flatness based feedforward action with a fractional pi regulator to control the intake valve engine. In: 2017 18th International Carpathian Control Conference (ICCC), pp. 456-461 (2017). https://doi.org/10.1109/CarpathianCC.2017.7970443  
19. Maaruf, M., Mysorewala, M.F.: Neuro-adaptive path following control of autonomous ground vehicles with input deadzone. Discov. Appl. Sci. 6(8), 415 (2024). https://doi.org/10.1007/s42452-024-06091-x  
20. Dong, H., Zhuang, W., Wu, G., Li, Z., Yin, G., Song, Z.: Overtaking-enabled eco-approach control at signalized intersections for connected and automated vehicles. IEEE Trans. Intell. Transp. Syst. 25(5), 4527-4539 (2024). https://doi.org/10.1109/TITS.2023.3328022  
21. Dong, H., Hu, Q., Li, D., Li, Z., Song, Z.: Predictive battery thermal and energy management for connected and automated electric vehicles. IEEE Trans. Intell. Transp. Syst. 26(2), 2144-2156 (2025). https://doi.org/10.1109/TITS.2024.3494734  
22. Yin, G., Chen, N., Li, P.: Improving handling stability performance of four-wheel steering vehicle via  $\mu$ -synthesis robust control. IEEE Trans. Veh. Technol. 56(5), 2432-2439 (2007). https://doi.org/10.1109/TVT.2007.899941  
23. Song, Z., Li, W., Yang, B., Ding, H., Huang, Y.: Enhanced control for handling and stability of 4wd electric vehicles with uncertain stability margins. EEE Trans. Transp. Electrif. 11(2), 6534-6547 (2025). https://doi.org/10.1109/TTE.2024.3510795  
24. Ding, F., Wang, R., Zhang, T., Zheng, G., Wu, Z., Wang, S.: Real-time trajectory planning and tracking control of bionic underwater robot in dynamic environment. Cyborg Bionic Sys. 5, 0112 (2024). https://doi.org/10.34133/cbsystems.0112  
25. Mercorelli, P.: Fuzzy based control of a nonholonomic carlike robot for drive assistant systems. In: 2018 19th International Carpathian Control Conference (ICCC), pp. 434-439 (2018). https://doi.org/10.1109/CarpathianCC.2018.8399669  
26. Mercorelli, P.: Using fuzzy pd controllers for soft motions in a car-like robot. Adv. Sci. Technol. Eng. Syst. J 3(6), 380-390 (2018). https://doi.org/10.25046/aj030646  
27. Mohammadzadeh, A., Taghavifar, H., Zhang, C., Alattas, K.A., Liu, J., Vu, M.T.: A non-linear fractional-order type-3 fuzzy control for enhanced path-tracking performance of autonomous cars. IET Control Theory Appl. 18(1), 40-54 (2024). https://doi.org/10.1049/cth2.12538  
28. He, Y., Liu, Y., Yang, L., Qu, X.: Deep adaptive control: Deep reinforcement learning-based adaptive vehicle trajectory control algorithms for different risk levels. IEEE Trans. Intell. Veh. 9(1), 1654-1666 (2024). https://doi.org/10.1109/TIV.2023.3303408  
29. Han, Z., Chen, P., Zhou, B., Yu, G.: Hybrid path tracking control for autonomous trucks: Integrating pure pursuit and deep reinforcement learning with adaptive look-ahead mechanism. IEEE Trans. Intell. Transp. Syst. pp. 1-15 (2025). https://doi.org/10.1109/TITS.2025.3530507  
30. Liang, J., Yang, K., Tan, C., Wang, J., Yin, G.: Enhancing high-speed cruising performance of autonomous vehicles through integrated deep reinforcement learning framework. IEEE Trans. Intell. Transp. Syst. 26(1), 835-848 (2025). https://doi.org/10.1109/TITS.2024.3488519

31. Beal, C.E., Gerdes, J.C.: Model predictive control for vehicle stabilization at the limits of handling. IEEE Trans. Control Syst. Technol. 21(4), 1258-1269 (2012). https://doi.org/10.1109/TCST.2012.2200826  
32. Li, Z., Chen, H., Liu, H., Wang, P., Gong, X.: Integrated longitudinal and lateral vehicle stability control for extreme conditions with safety dynamic requirements analysis. IEEE Trans. Intell. Transp. Syst. 23(10), 19285-19298 (2022). https://doi.org/10.1109/TITS.2022.3152485  
33. Chen, L., Liu, Y., Sun, Z., Yang, L.: Path tracking control for connected automated vehicle using lateral stability envelope at handling limits. In: IEEE International Conference on Intelligent Transportation Systems (ITSC), pp. 3641-3646 (2023). https://doi.org/10.1109/ITSC57777.2023.10422012  
34. Zhou, C., Liu, X.h., Xu, F.x.: Intervention criterion and control strategy of active front steering system for emergency rescue vehicle. Mech. Syst. Sig. Process. 148, 107160 (2021)  
35. Liang, Y., Li, Y., Yu, Y., Zheng, L.: Integrated lateral control for 4wid/4wis vehicle in high-speed condition considering the magnitude of steering. Veh. Syst. Dyn. 58(11), 1711-1735 (2020)  
36. Di Cairano, S., Tseng, H.E., Bernardini, D., Bemporad, A.: Vehicle yaw stability control by coordinated active front steering and differential braking in the tire sideslip angles domain. IEEE Trans. Control Syst. Technol. 21(4), 1236-1248 (2012). https://doi.org/10.1109/TCST.2012.2198886  
37. Li, X., Xu, N., Guo, K., Huang, Y.: An adaptive smc controller for evs with four iwms handling and stability enhancement based on a stability index. Veh. Syst. Dyn. 59(10), 1509-1532 (2021). https://doi.org/10.1080/00423114.2020.1767795  
38. Rajamani, R.: Vehicle dynamics and control. Springer Science & Business Media (2011). https://doi.org/10.1007/978-1-4614-1433-9  
39. Bobier, C.G., Gerdes, J.C.: Staying within the nullcline boundary for vehicle envelope control using a sliding surface. Veh. Syst. Dyn. 51(2), 199-217 (2013). https://doi.org/10.1080/00423114.2012.720377

40. Zhang, Z., Yu, J., Huang, C., Du, R.: Coordinated torque distribution method of distributed drive electric vehicle to reduce control intervention sense. Veh. Syst. Dyn. 62(1), 198-221 (2024). https://doi.org/10.1080/00423114.2023.2200190  
41. Ferramosca, A., Limon, D., Alvarado, I., Alamo, T., Camacho, E.: Mpc for tracking with optimal closed-loop performance. In: 2008 47th IEEE Conference on Decision and Control, pp. 4055-4060 (2008). https://doi.org/10.1109/CDC.2008.4739089  
42. Xu, N., Huang, Y., Askari, H., Tang, Z.: Tire slip angle estimation based on the intelligent tire technology. IEEE Trans. Veh. Technol. 70(3), 2239-2249 (2021). https://doi.org/10.1109/TVT.2021.3059432  
43. Song, Z., Ding, H., Wang, J., Huang, C., Guo, Y.: Research on the strategy of cruise control system for urban traffic jam assistant. P I M ECH ENG D-J AUT p. 09544070241283886 (2024). https://doi.org/10.1177/09544070241283886  
44. Ding, H., Song, Z., Zhang, J., Xu, N., Gao, S., Zhao, C.: Efficient motion planning and control for automated lane change considering road adhesion coefficient. Int. J. Veh. Des. 95(3-4), 320-347 (2024). https://doi.org/10. 1504/IJVD.2024.139179

Publisher's Note Springer Nature remains neutral with regard to jurisdictional claims in published maps and institutional affiliations.

Springer Nature or its licensor (e.g. a society or other partner) holds exclusive rights to this article under a publishing agreement with the author(s) or other rightsholder(s); author self-archiving of the accepted manuscript version of this article is solely governed by the terms of such publishing agreement and applicable law.