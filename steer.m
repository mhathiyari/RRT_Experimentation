 function q_possible = steer(q_new,q_nearest)
    q_new.coord = round(q_new.coord,3);
    q_new.cost = q_new.cost;
    
    distance = inf;
    best_angle = 0 ; 
    steering_max = 0.58904862;
    steering_inc = 0.0117809724;

    for s = -steering_max:steering_inc:steering_max
    [q_f] = new_state(q_nearest,s);
    new_distance = distance_euc(q_new.coord,q_f.coord); % doesnt look at theta diff for now
    if distance > new_distance
       distance = new_distance;
       q_possible.cost = distance;
       q_possible.input = s;
       q_possible.coord = q_f.coord;
    end
    end
        
 end
 
 
  function d = distance_euc(x1,x2)
 d = sqrt((x1(1)-x2(1))^2+(x1(2)-x2(2))^2);
 
  end
 
 function [q_f] = new_state(q_nearest,s)
 %RK4
 dt = 0.5;
 k1  = dynamics(q_nearest.coord,s);
 k2 = dynamics(q_nearest.coord+k1.coord/2,s);
 k3 = dynamics(q_nearest.coord+k2.coord./2,s);
 k4 = dynamics(q_nearest.coord+k3.coord,s);
 
 q_f.coord = q_nearest.coord + dt/6*(k1.coord+2*k2.coord+2*k3.coord+k4.coord);
 
 end
 
 function x_dot = dynamics(x,u)
mass = 760;
lf = 1.025;
lr = 0.787;
inertia = 1490.3;
cf = 5146/2;
cr = 3430/2;
speed = 21.1;
 
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
 