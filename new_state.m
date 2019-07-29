function [q_f,point_list] = new_state(q_nearest,s)
 %RK4
 dt = 0.01;
 point_indx = 1;
 for i = 0:dt:0.5
 k1  = dynamics(q_nearest.coord,s);
 k2 = dynamics(q_nearest.coord+k1.coord/2,s);
 k3 = dynamics(q_nearest.coord+k2.coord./2,s);
 k4 = dynamics(q_nearest.coord+k3.coord,s);
 
 q_nearest.coord = q_nearest.coord + dt/6*(k1.coord+2*k2.coord+2*k3.coord+k4.coord);
 point_list(point_indx,:) = q_nearest.coord;
 point_indx = 1 + point_indx;
 end
 q_f.coord = q_nearest.coord;
 
 end
 
 function x_dot = dynamics(x,u)
mass = 760;
lf = 1.025;
lr = 0.787;
inertia = 1490.3;
cf = 5146/2;
cr = 3430/2;
speed = 10.1;
  
%  x =  x.coord(1);
%  y = x.coord(2);
 theta = x(3);
 vy = x(4);
 r = x(5);
 
 cosInput = cos(u);
 cosTheta = cos(theta);
 sinTheta = sin(theta);

 a = -(cf*cosInput+cr)/(mass*speed);
 b = (-lf*cf*cosInput+lr*cr)/(mass*speed)-speed;
 c = (-lf*cf*cosInput+lr*cr)/(inertia*speed);
 d = -(lf*lf*cf*cosInput+lr*lr*cr)/(inertia*speed);
 e = cf*cosInput/mass;
 f = lf*cf*cosInput/inertia;

 vyDot = a*vy + c*r + e*u;
 rDot = b*vy + d*r + f*u;
 xDot = speed*cosTheta - vy*sinTheta;
 yDot = speed*sinTheta + vy*cosTheta;
 thetaDot = r;
 
 
x_dot.coord(1) = xDot;
x_dot.coord(2) = yDot;
x_dot.coord(3) = thetaDot;
x_dot.coord(4) = vyDot;
x_dot.coord(5) = rDot;
 
 end