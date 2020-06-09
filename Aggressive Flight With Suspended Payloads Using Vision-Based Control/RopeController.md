# 小结

![image-20200609150215719](Aswithrope.assets/image-20200609150215719.png)

首先我们整个系统的输入就是负载的期望位置$\bold x_{L,des} \in \R^3$，前提是我们默认了无人机的yaw角是不变的。

注意，控制器中$()_c,()_{des}$的变量都是规划变量，无角标的变量都是反馈（实际）变量。

### 负载位置控制器

负载位置控制器收到的反馈是负载的当前位置和当前速度。输入是负载的期望位置，通过变量平滑我们可以计算出绳子的拉力方向向量
$$
\bold p = -\frac{(\bold{\ddot x}_L+g\bold{e_z^\mathcal I})}{||(\bold{\ddot x}_L+g\bold{e_z^\mathcal I})||_2} \tag{1}
$$

> 注意这个变量是基于力学的，实际使用时应当带入反馈负载加速度而不是期望。

我们首先估计一个中间向量
$$
A = (m_Q+m_L)\left ( 
-\bold k_x \bold e_{x_L} - -\bold k_v \bold e_{\dot x_L} -  \bold k_i \int \bold e_{x_L} dt
\right )\\
+(m_Q+m_L)(\bold {\ddot x}_L + g \bold e_z^{\mathcal I})+ m_Q l(\bold {\dot p}\cdot \bold {\dot p})\bold p \tag{2}
$$
其中$\bold k_x ,\bold k_v ,\bold k_i $分别是对角的增益矩阵，$\bold e_{x_L} = \bold x_L - \bold x_{L,des}, \bold e_{\dot x_L} = \bold{\dot x_L} - \bold{\dot x_{L,des}}$。由此可以得到控制负载旋转的命令
$$
\bold p_c = -\frac{\bold A}{||\bold A||_2} \tag {3}
$$

> 公式1怀疑是原文打错了，此处已经修正过了 原文是ki减积分

对该变量进行简单求导就可以的到$\bold { \dot p_c}$。

### 负载姿态控制器

在上一节中我们介绍了如何获得$\bold { p_c}, \bold { \dot p_c}$，接下来我们以这两个两进行输入，设计负载姿态控制器

首先我们计算$\bold F$
$$
\bold F = m_Ql(-k_p\bold e_p - k_{\dot p }\bold e_{\dot p}+ \left (\bold p\cdot (\bold p_c \times \bold{\dot p}_c)\right ) 
(\bold p\times \bold {\dot p}) \\
+(\bold p_c \times \bold{\ddot p_c} )\times \bold p ) - (\bold p_c\cdot \bold p)\bold p_c\tag 4
$$
其中误差函数定义为
$$
\bold e_p = \bold{\hat  p}^2\bold p_c ,\bold{e }_{\dot p} = \bold{\dot p}-(\bold p_c \times \bold{\dot p}_c)\times \bold p \tag {5}
$$
随后由$\bold F$计算控制旋转
$$
\bold R_c = \left[ 
\frac{\bold b_{3c}\times \bold c_{1c}}{||\bold b_{3c}\times \bold c_{1c}||_2} \times

\bold b_{3c}\frac{\bold b_{3c}\times \bold c_{1c}}{||\bold b_{3c}\times \bold c_{1c}||_2}
\bold b_{3c}
\right ]\\
\hat \Omega_c = R_c^T\dot R_c\tag{6}
$$
其中
$$
\bold b_{3c} = \frac{\bold F}{||\bold F||_2},\bold c_{1c} = [\cos(\psi) \ \ \sin(\psi ) \ \ 0 ]^T \tag 7
$$
由此我们就得到了$\bold R_c,\bold\Omega_c,\bold F$

### 无人机姿态控制器

到这里我们就回到了熟悉的无人机控制部分，只是形式稍有不同。无人机姿态控制器
$$
\begin{aligned}
M =& -\bold k_R\bold e_R- \bold k_\Omega\bold e_\Omega-\\
&+ \bold \Omega\times \mathbb I \Omega - \mathbb I (\hat \Omega \times \bold R^T\bold R_c\bold \Omega_c - \bold R^T\bold R_c\bold \Omega) 
\\
f =& \bold F \cdot R \bold e_z^{\mathcal I}
\end{aligned}


\tag{8}
$$
其中误差函数表示为
$$
\bold e_R = \frac{1}{2}(\bold R_c ^T \bold R - \bold R ^T \bold R_c )^\bigvee,
\bold e_{\Omega} = \bold\Omega - \bold R^T\bold R_c\bold \Omega_c \tag 9
$$


### 控制分配

现在我们已经得到了无人机的$f,M$，最后使用之前的控制分配算法分配给无人机电机即可。