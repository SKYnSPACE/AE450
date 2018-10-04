<br>

# [AE450] Scoring rubric for the Homework Assignment #1

## 0. Disclaimer
>  1. This guideline may contain some typo and errors.
>  2. Although we use MATLAB to do some dirty works, it is highly recommended to see the behind of the curtain(e.g. how to make the controllability matrix, Ackermann's formula, Drawing Bode plot, etc.).  
>  3. There may several other manageable approaches, solutions, and answers to a single problem(e.g. ways of system stability proof, gains or pole locations in feedback controller designs, etc.).
>  Therefore, all reasonable solutions may take full credits.
>  IN OTHER WORDS, HOWEVER, THIS ALSO MEANS GIVING THE SAME ANSWERS ON THE PROBLEMS [2d], [4b], [4c], and [4d] CAN BE CONSIDERED AS A SUSPICIOUS ACT BETWEEN STUDENTS, AND NEED JUSTIFICATION TO TAKE FULL SCORES.
> 4. If you run into any troubles regarding the assignment #1 (error reporting, claims, etc.), ask T.A. in charge for the help. Assistant: Seongheon Lee (skynspace@kaist.ac.kr)
```
>> NOTICE
% Shaded area describes a programming code snippet or outputs from the programming code.
% All codes and scripts are written in MATLAB.
% You may need to install some libraries regarding control theories (e.g. control system toolbox).
% Scripts are also available @[https://github.com/SKYnSPACE/AE450HW1]
% Take the hyperlink below.
```
<s>[https://github.com/SKYnSPACE/AE450HW1](https://github.com/SKYnSPACE/AE450HW1)</s> (Not available before 10/OCT/2018)

- **[VERSION CONTROL]** 
[Ver. 1.0 @03/OCT/2018] Published the first edition. 

## 1. Warmup 
### &nbsp; Give two examples of feedback control systems in daily life. Define the inputs and outputs of the system, underlying system state and dynamics, and describe the objective function. Describe the uncertainties in the system, and noise and disturbances that are likely experienced. (20pts.)

> No fixed solution to the given problem. Gets full credit as long as you come up with two proper feedback control systems and fully described their **(1) inputs, (2) outputs, (3) states, (4) dynamics, (5) objective functions, (6) uncertainties, and (7) noise and disturbances**.

&nbsp; One simple possible solution can be described as follows:
Mass-Spring-Damper System|Descriptions
-|-
Input| $u$: Force acting on the mass.
Output| $x$: Position of the mass.
States| $X=[x, \dot{x}]$: Position and velocity of the mass.
Dynamics| $m\ddot{x}+c\dot{x}+kx=u$
Objective function| Reducing error between the desired position and actual position.
Uncertainties| Parametric uncertainties included in the mass, damping coefficient, and spring coefficient.
Noise| Measurement noise included in the position and/or velocity sensor.
Disturbances| Other external forces, such as wind, frictions, etc.

<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
For every missed description|-1| In the worst case, 10 points will be deducted for each system
System example not given|-10| per example.

## 2. State-Space Control Analysis
### &nbsp; Given an inverted pendulum (neglect the mass and inertia of a bar) on a cart, (linearlized) mathematical model of the system can be described as follows:

$$
\begin{aligned}
Ml\ddot{\theta} &=(M+m)g\theta-u &(2.1)\\
M\ddot{x} &= u - mg\theta &(2.2)
\end{aligned}
$$

<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/Htn6qAjoYKbAc3EmNCylbn61zgcCJclLLNMjqAuUpgmSNrZdDRhIPcv9zv6ECmbqYMZB7VNfBGY=s300"></img> <br> <em>Figure1. Inverted Pendulum on a Cart</em></p>

### &nbsp; Please answer to the following questions when the system parameters are given to be;
$$
M = 2kg,\quad m = 0.1kg,\quad  l = 0.5m,\quad g=9.81m/s^2.
$$


### [2a]. Derive a state-space equation of the form $\bold{\dot{x}}=\bold{Ax}+\bold{B}u$, where state variable $\bold{x}=[\theta, \dot{\theta}, x, \dot{x}]^T$. (5pts.)

&nbsp; Since $\bold{\dot{x}}=[\dot{\theta}, \ddot{\theta}, \dot{x}, \ddot{x}]^T$, rearranging equation (2.1), and (2.2) yields,
$$
\begin{aligned}
\dot{\theta}=&1\cdot{\dot{\theta}}&(2a.1)&\\
\ddot{\theta}=&{{(M+m)g}\over{Ml}}\cdot{\theta}-{{1}\over{Ml}}\cdot{u} &(2a.2)&\\
\dot{x}=&1\cdot{\dot{x}}&(2a.3)&\\
\ddot{x}=&-{{mg}\over{M}}\cdot{\theta}+{{1}\over{M}}\cdot{u}&.(2a.4)&
\end{aligned}
$$
&nbsp; Organizing (2a.1)-(2a.4) into a matrix form yields,
$$
\begin{aligned}
\begin{bmatrix}
\dot{\theta} \\ \ddot{\theta} \\ \dot{x} \\ \ddot{x}
\end{bmatrix} =&
\begin{bmatrix}
0 & 1 & 0 & 0 \\
{{(M+m)g}\over{Ml}} & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
-{mg}\over{M} & 0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
{\theta} \\ \dot{\theta} \\ x \\ \dot{x}
\end{bmatrix} +
\begin{bmatrix}
0 \\ -{{1}\over{Ml}} \\ 0 \\ {1}\over{M}
\end{bmatrix} u &.(2a.5)&
\end{aligned}
$$
&nbsp; Lastly, putting the system parameters into (2a.5) completes the form:
$$
\begin{aligned}
A =&
\begin{bmatrix}
0 & 1 & 0 & 0 \\
20.601 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
-0.4905 & 0 & 0 & 0
\end{bmatrix}
, B=
\begin{bmatrix}
0 \\ -1 \\ 0 \\ 0.5
\end{bmatrix} &.(2a.6)&
\end{aligned}
$$
```
% SCRIPT
M=2.0;
m=0.1;
l=0.5;
g=9.81;
A = [0 1 0 0; ...
(M+m)*g/M/l 0 0 0;...
0 0 0 1; ...
-m*g/M 0 0 0]
B = [0; -1/M/l; 0; 1/M]
```
```
% OUTPUTS
A =

0    1.0000         0         0
20.6010         0         0         0
0         0         0    1.0000
-0.4905         0         0         0


B =

0
-1.0000
0
0.5000
```
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Failed to build the state-space equation|-3|(2a.5)
Miscalculation|-2|(2a.6)



### [2b]. Give an analysis of the stability of the system. (5pts.)
&nbsp; One way to check the stability of a system from a given state-space equation is calculating eigenvalues of the A matrix. In MATLAB, you can use eig(A) to produce the eigenvalues of a square matrix A.
```
% SCRIPT
eig(A)
```
```
% OUTPUTS
ans =

0
0
4.5388
-4.5388
```
&nbsp; Since one of the eigenvalue is located on the right-half plane of a complex plane, the given system is naturally unstable.
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Incomplete stability analysis|-5| 


### [2c]. Give an analysis of the controllability of the system if you can access to the full state of the system. (10pts.)
&nbsp; The easiest way of checking the controllabililty of the given system is to check the full rank of a controllability matrix(our system has the dynamics matrix with the dimension of $n=4$, and input matrix dimension of $m=1$).
$$
M_C=[B \ |\  AB \ |\  A^2B \ |\  \cdots \ |\  A^{n-1}B] \\
(A \in \mathbb{R}^{n\times n}, \ B\in \mathbb{R}^{n\times m}) 
$$
&nbsp; Building and checking the rank of controllability can be done in a single line.

```
% SCRIPT
rank(ctrb(A,B))
```
```
% OUTPUTS
ans =

4
```
&nbsp; Since the controllability matrix has the full rank, our system is controllable (ONLY WITH A SINGLE INPUT)!
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Wrong approach to the controllability|-10| 
Mistakes on calculations|-5 |yet, with a right approach


### [2d]. Design a full-state feedback controller(regulator) with the form of $u=-\bold{Kx}$ if the system is controllable. If not, describe the necessary conditions to be controllable. (10pts.)

&nbsp; We want to use the input $u$ to modify the unstable eigenvalues of $A$ to change the system dynamics.
Since we assumed a full-state feedback, $u$ has the form of $u=r-\bold{Kx}$, where $r$ is some reference input and the gain $\bold{K}$ is in $\mathbb{R}^{1\times n}$. When $r=0$, we call this controller a regulator.
<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/fmb6Na_TOmHgiFR-4ofH-AZf5xhnOpZwfDIiMQT1g8cTqNcpxLF4vP-YWETOQP3WnyVGm7hA3Aem=s300"></img> <br> <em>Full-state feedback control</em></p>

&nbsp; Therefore the closed-loop dynamics can be described as follows:

$$
\begin{aligned}
\bold{\dot{x}}=&\bold{Ax}+\bold{B}u =\bold{Ax}+\bold{B}(r-\bold{Kx})\\
=&(\bold{A}-\bold{BK})\bold{x}+\bold{B}r \\
=&\bold{A_{_{cl}}x}+\bold{B}r \\
y=&\bold{Cx}=\bold{x}
\end{aligned}
$$

&nbsp; Now, our objective can be explained as follows:
- We want to make $\bold{A_{cl}}$ stable although $\bold{A}$ is unstable.
- Pick $\bold{K}$ such that $\bold{A_{cl}}$ has the desired properties.
- Put the poles of $\bold{A_{cl}}$ at the desired locations. 

&nbsp; One may solve the $det(sI-\bold{A_{cl}})=0$ to put eigenvlaues anywhere in the left half of the complex plane (assuming complex conjugate pairs of poles), a MATLAB function 
```
>> K=acker(A,B,P) % P: Desired Pole locations.
```
gives us a method of doing this entire design process in one single step. (cf. Ackermann's Formula)

&nbsp; For example, one can design the controller $\bold{K}$ that makes $\bold{A_{cl}}$ to take $-5, -6, -7, -8$ as pole locations:
```
% SCRIPT
P = [-5 -6 -7 -8];
K = acker(A,B,P)
A_cl = A-B*K;

C = eye(4);
D = 0;

sys_cl = ss(A_cl,B,C,D);
figure(1)
step(sys_cl); grid on;
```
```
% OUTPUTS
K =

-357.2279  -80.3323 -171.2538 -108.6646
```

<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/xQI0vubvwkrmPtog8hdTmH9flR0pW7Rm8wnDbZh7DZCjmRJrOcJK_u2IucbWrCywMl6J_OQkHRNP=s600"></img> <br> <em>Prob2. figure(1)</em></p>
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Wrong approach to the controller design|-10| 
Mistakes on calculations|-5 |yet, with a right approach

## 3. Frequency Response Control Analysis

### &nbsp; Consider a feedback system composed of a plant of $P(s)={100\over s}$ , and a controller of $C(s)={100\over {s+40}}$
<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/7Ge3G1BIDDZrR8ezlMEbGvhTRKZf7-cZTsPMVXtHEg3LUUy7dTKCR7nRGtZudiHE8v_Z4MnBwy8=s600"></img> <br> <em>Figure2. Feedback Control Loop</em></p>

&nbsp; Given the transfer functions, MATLAB function 'tf(NUM,DEN)' creates a continuous-time transfer function with numerator NUM, and denominator DEN. 
```
% SCRIPT
P = tf([100],[1 0])     % 100/s
C = tf([100],[1 40])    % 100/(s+40)
```

### [3a]. Draw the Bode plots of the loop transfer function (i.e., $L(s)=P(s)C(s)$). (5pts.)
&nbsp; Building other transfer functions from the given blocks, and drawing a bode plot is also quite intuitive.
```
% SCRIPT
L = P*C
figure(1)
bode(L); grid on;
```
<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/pUHHwwzqELE8lyAD36lgfCCRmnmMVcZNUIJxEFxbOtIgzkei0M9h2yjA9Ml-kfQI_jYksSwjFt_P=s600"></img> <br> <em>Prob3. figure(1)</em></p>
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Wrong LTF|-1| 
Wrong, or missed magnitude diagram|-2 |
Wrong, or missed phase diagram|-2 | 

### [3b]. Calculate the gain and phase margins from the plots in [3a]. (5pts.)
```
% SCRIPT
[GM, PM] = margin(L)
```
```
% OUTPUTS
GM =

Inf

PM =

22.6028
```
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Wrong G.M.|-2| 
Wrong P.M.|-2|
Wrong G.M. & P.M.|-5| 

### [3c]. Draw the Nyquist plot and analyze the closed-loop stability. (5pts.)
```
% SCRIPT
figure(2)
set(gcf, 'position', [0, 0, 560, 900])
subplot(2,1,1)
nyquist(L); grid on; axis equal;
subplot(2,1,2)
nyquist(L); grid on; axis([-3 3 -3 3]);
```
<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/0Ym9XGpp3d1OyyZcJoueNxAX8IaR1IzKNCwcPzPWoIIAy_sHrGjfpViXleqIYd09B73gJaQzLOD7=s900"></img> <br> <em>Prob3. figure(2)</em></p>

&nbsp; In this polt, the number of unstable open-loop poles is zero ($P = 0$), the number of clockwise encirclement of (-1) is also zero ($N=0$). Therefore, the number of unstable closed-loop poles is $Z=P+N=0$, which means the closed-loop stability is garunteed.
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Wrong Nyquist plot|-2| 
Wrong, or missed analysis|-2|
Did nothing|-5| 

### [3d]. Show the step response of the closed-loop system. (5pts.)
&nbsp; Slight abuse of the notations let us find the closed-loop transfer function from the reference input $r$ to output $y$:
$$
\begin{aligned}
y=&P \cdot u \\
=&PC \cdot e  = L \cdot e\\
=& L \cdot (r-y) \\
(1+L) \cdot y =& L \cdot r \\
{y \over r} =& {{L}\over{1+L}} = T
\end{aligned}
$$

&nbsp; MATLAB script to show the response is as follows:
```
% SCRIPT
T = L/(1+L)
figure(3)
step(T); grid on;
```
<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/SqSp-CAY3eI9PrPRg1aHb9EHSJnKmyefYOjI-qQXoj6I8PUkgI3FAaWqvIXhDmtpixybew-Vs4gj=s600"></img> <br> <em>Prob3. figure(3)</em></p>
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Wrong complementary sensitivity function (T)|-2| 
Wrong, or missed step response|-2|
Did nothing|-5| 


## 4. Basic Root Locus Analysis
### &nbsp; Please answer to the following questions with a given feedback control loop.
<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/sNcs7HSnmPsn_xI2AnmFpI_sR1fzKMOZbYj8x0-kRTOgW66VS1ZMG_FlcjExdWHAv4upECDfKtM=s600"></img> <br> <em>Figure3. Lead compensator</em></p>

### [4a]. Draw root locus with respect to the different pole ($p$) placement stratagies. (5pts.)
```
% SCRIPT
p = [1 3 6 9 12 20];
figure(1)
set(gcf,'position', [0, 0, 900, 600]);
for i=1:length(p)
G = tf([1],[1 0 0]);    % 1/s^2
C = tf([1 1],[1 p(i)]); % (s+1)/(s+p)
subplot(2,3,i)
rlocus(G*C); grid on;
title(['p = ' num2str(p(i)) ''])
end
```
<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/r4GqIcjh-vYaVpXjdDnO6huy0qhc6xK1jKlicSbRehCCkUNlZVkbALXGPA6xxn1W16i0JBG25Rsa=s900"></img> <br> <em>Prob4. figure(1)</em></p>

&nbsp; To design a lead compensator, one should grasp the tendency due to the different pole placement strategies. In the figure above, we can see three different tendencies with repect to the pole locations (e.g. $p=1$, $1<p<9$, and $p \geq 9$).


-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Choise of 'p' are not enough to properly analyze the system|-3| More than two shapes in (p>1) must be presented.
Did nothing|-5| 

### [4b]. Select values of $p$ and $k$ to make the closed-loop step response satisfies the following conditions. (10pts.)
- Overshoot < 16.3[%],
- 2% Settling Time < 1.667[sec]

&nbsp; From [4b] to [4d], we can use MATLAB sisotool to find the satisfactory $p$ and $k$ conditions.
&nbsp; One can start from the scratch and put the design requirements on the root locus and step response: 
```
% SCRIPT
G = tf([1],[1 0 0]);  % 1/s^2
C = tf([1 1],[1 20]); % (s+1)/(s+20)
sisotool(G, C);
```
&nbsp; Or start from the saved file in <s>[https://github.com/SKYnSPACE/AE450HW1](https://github.com/SKYnSPACE/AE450HW1)</s> (Not available before 10/OCT/2018) containing all the design requirements: 
```
% SCRIPT
sisotool('HW1_Prob4.mat');
```
&nbsp; Drag around the poles (square box) on the interactive root locus editor, and see whether the step response is within the boundary. For example, I choosed $k=372.67$, $p=30$ in the saved 'mat' file.
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Overshoot not satisfied|-5| cross-checked with the prob [4c], and [4d].
Settling time not satisfied|-5| cross-checked with the prob [4c], and [4d].

### [4c]. Show the poles for the $k$ that you found in [4b]. (5pts.)
For the example 'mat' file with $k=372.67$, $p=30$, we have closed-loop pole locations as follows:
$$
\begin{aligned}
s_{1,2}=& -14.5 \pm 11.5i \\
s_3 =& -1.09
\end{aligned}
$$

<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/2OYX4n9krLfXp6VnJISIPSlLYqftMBlc2_6NKV_aQ5DOFpbm6CriWlamsOBlLfgz3thPjy8bTDV3=s600"></img> <br> <em>Prob4. figure(2)</em></p>
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Not appropriate poles given|-5| poles mismaching with the results in [4b] 

### [4d]. Show the step response of the designed closed-loop system in [4b]. (5pts.)
<p align="center"><img class="pull-left" src="https://lh3.googleusercontent.com/RZPDg7M0_XvpOwIdvf4tx-7r_IFNOOdClAaPdWeGuWA0AD_ZFI-hHF1zakLKHd9SDo9qbYXVcrTX=s600"></img> <br> <em>Prob4. figure(3)</em></p>
<br>

-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Not appropriate response for the suggesting p and k|-5| step response mismatching with the results in [4b]

### [4e]. Discuss on the steady-state error characteristics when the $r(t)$ is a unit ramp input (i.e., $r(t) = t$ for $t>=0$, and $r(t) = 0$ for $t<0$). (5pts.)

&nbsp; The transfer function from $r$ to $y$ can be described as follows (look for the problem [3d] if you need the detailed explanation):
$$
{{Y(s)}\over{R(s)}} = {{ks+1}\over{s^3+ps^2+ks+k}}
$$

&nbsp; For the ramp input, $R(s)=\mathcal{L}(r(t))={{1}\over{s^2}}$. Putting this into the above equation yields;
$$
Y(s) = {{ks+1}\over{(s^3+ps^2+ks+k)\cdot s^2 } \ .}
$$

&nbsp; Therefore, the error signal can be expressed as
$$
E(s) = R(s) - Y(s) = \left( 1-{{ks+1}\over{(s^3+ps^2+ks+k)}} \right) \cdot {{1}\over{s^2}} = {{s^2(s+p)}\over{s^3+ps^2+ks+k}}\cdot {{1}\over{s^2}} \ .
$$

&nbsp; Lastly, the final value theorem gives the steady-state error characteristics as follows:
$$
\lim_{t \to \infin}e(t)= \lim_{s \to 0}s \cdot E(s)=0
$$
$\therefore$ We can expect the zero steady-state error.


-  SCORING CRITERIA

Criteria|Points|Notes
-|-|-
Wrong answer|-5| 
