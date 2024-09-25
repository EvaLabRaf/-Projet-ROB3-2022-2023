# Kalman Filter sub project

For the sensor fusion project we choose to implement the Kalman filter directly to the main project of our 5th and last year in Polytech Nice Sophia.

<br />

we are going to apply it on the GPS and or laser 

The goal of the project is to reduce the noise on the GPS data to have a better precision of the position.

We will combine the GPS data with the robot movement data.

we need one model of the movement prediction and one measuring model to read the sensor information.

We need to define a state vector which have to be composed by the positions of the axis x and y ; and the speed of the axis x and y as well.

The state-vector can be wrote like :

$$
X_k =
\left(\begin{array}{cc} 
 x_k\\
y_k\\
v_{x_k}\\
v_{y_k}\\
\end{array}\right)
$$ 

First we have to obtain the movement model which describe the state changing during the time.

We can firstly estimate the movement as a linear uniform movement to see what data we get and then try to use a more realist model.

So we can use the equation : 

$$
X_{k+1} = F * X_k + u
$$

With 

$$
F = 
\left(\begin{array}{cc} 
 1 && 0 && \Delta t && 0\\
0 && 1 && 0 && \Delta t\\
0 && 0 && 1 && 0\\
0 && 0 && 0 && 1
\end{array}\right)
$$

The vector u will be the noise vector or the acceleration as well.

Then we have to get the measuring model ; the GPS will give us the positions of the robot

$$
z_t =
\left(\begin{array}{cc} 
 x_{gps}\\
y_{gps}\\
\end{array}\right)
$$ 

The equation which is related to the state vector is : 

$$
Z_k = H * X_k
$$

With H equals to : 

$$
H =
\left(\begin{array}{cc} 
 1 && 0 && 0 && 0\\
0 && 1 && 0 && 0\\
\end{array}\right)
$$

We can now use the Kalman Filter equation seen in class which are : 

$$
\widehat{X_{k+1}} = F * X_k
$$

$$
P_{k+1} = F * P_k * F^\intercal + Q
$$

Ones we had a GPS data, we can estimate the previous one with the Kalman gain : 

$$
K_k = P_k * H^\intercal * (H * P_k * H^\intercal + R)^{-1}
$$

Then we update the state :

