function RRT_v2_musti()
    clc
    close all

    width = 10;
    height = 10;

    origin = [3,3];
    goal = [8,8];
    % box
    obstacle = zeros(4,2,2);
    obstacle(1,:,:) = [1,1;-1,1];
    obstacle(2,:,:) = [-1,1;-1,-1];
    obstacle(3,:,:) = [-1,-1;1,-1];
    obstacle(4,:,:) = [1,-1;1,1];
    offset = 0;
    obstacle = obstacle + ones(4,2,2)*offset;

    iterations = 1000  ;
    vertecies = origin; 
    q_start.coord = origin;
    q_start.cost = 0;
    q_start.parent = origin;
    q_goal.cost = 0;
    q.goal.coord = goal;
    nodes(1) = q_start;
   
    vert_count = 1;
    edge_count = 0;
    %% Algorithm
     for i = 1:iterations
         q_new.coord = random_point(width,height);
         [q_nearest,q_new] = v_nearest(q_new,nodes);
         q_new = steer(q_new,q_nearest);
         if collision_check(q_new.coord,q_nearest.coord,obstacle)
             q_new.parent = q_nearest.coord;
             nodes = [nodes q_new];
         end
     end
%% Plot code
     for i = 1:length(nodes)
     vertex(i,:) = nodes(i).coord;
     parent(i,:) = nodes(i).parent;
     edges.x(i,:) = [vertex(i,1),parent(i,1)];
     edges.y(i,:) = [vertex(i,2),parent(i,2)];
     end
     
    figure('name', 'RRT basic');
    scatter(origin(1), origin(2), 45, '*','r','LineWidth',1); hold on;
    scatter(vertex(:,1), vertex(:,2), 10,linspace(1,10,length(vertex(:,1))),'filled'); hold on;
    plot(edges.x', edges.y');
    plot(obstacle(:,:,1),obstacle(:,:,2))

end


function [p_rand] = random_point(width,height)
 offset = [0,0] - [width, height]./2;
 x_rand = width*rand()+offset(1);
 y_rand = height*rand()+offset(2);
 p_rand = [x_rand,y_rand];
end

 function [q_nearest,q_new] = v_nearest(q_new,nodes)
 p_rand = q_new.coord;
 q_new.cost = sqrt((p_rand(1,1)-nodes(1).coord(1,1))^2+(p_rand(1,2)-nodes(1).coord(1,2))^2);
 q_nearest.coord = nodes(1).coord;
 n = size(nodes);
 for i = 1:n(2)
   v  = nodes(i);
   new_dist = sqrt((p_rand(1,1)-v.coord(1))^2+(p_rand(1,2)-v.coord(2))^2);
   if new_dist< q_new.cost
       q_new.cost = new_dist;
       q_nearest.coord = v.coord;
   end
 end
 end
 
 function q_new = steer(q_new,q_nearest)
    q_new.coord = round(q_new.coord,3);
    q_new.cost = q_new.cost;
    
 end
 % taken from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 function collide = collision_check(p_new,p_nearest,obstacle) %has to be a better way to do this 
%  collide = 1; 
%  return   
    for i = 1:length(obstacle)
         p1 = [obstacle(i,1,1),obstacle(i,2,1)];
         q1 = [obstacle(i,1,2),obstacle(i,2,2)];
         o1 = orientation(p_nearest,p_new,p1);
         o2 = orientation(p_nearest,p_new,q1);
         o3 = orientation(p1,q1,p_nearest);     
         o4 = orientation(p1,q1,p_new);

        if (o1 ~= o2 && o3 ~= o4)
            collide = 0;
            return;
        end
         collide = 0;
         if (o1 == 0 && onsegment(p_nearest,p1,p_new))
             return;
         end
        if (o2 == 0 && onsegment(p_nearest,q1,p_new))
         return;
        end
        if (o3 == 0 && onsegment(p1,p_nearest,q1))
         return;
        end
        if (o4 == 0 && onsegment(p1,p_new,q1))
         return;
        end
        collide = 1;
     end
 end
 
 function val = orientation(p, q, r) %need to readup on this
     val = (q(1,2)- p(1,2)) * (r(1,1) - q(1,1)) - ...
              (q(1,1) - p(1,1)) * (r(1,2) - q(1,2));
     if val == 0
         return
     end
     if val>0
         val =1;
     else
         val = 2;
     end
 end

 function val = onsegment(p, q, r)%need to readup on this
    if (q(1,1) <= max(p(1,1), r(1,1)) && q(1,1) >= min(p(1,1), r(1,1)) && ...
        q(1,2) <= max(p(1,2), r(1,2)) && q(1,2) >= min(p(1,2), r(1,2))) 
        val = 1;
    else
        val = 0;
    end

 end
 
