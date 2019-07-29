function RRT_Star_musti()
    clc
    close all

    width = 1000;
    height = 1000;

    origin = [103,150,0,0,0];%[x,y,theta,vy,r]
    goal = [-400,-400,0,0,0];
    % box
    obstacle = zeros(4,2,2); 
    obstacle(1,:,:) = [1,1;-1,1];
    obstacle(2,:,:) = [-1,1;-1,-1];
    obstacle(3,:,:) = [-1,-1;1,-1];
    obstacle(4,:,:) = [1,-1;1,1];
    offset = 0;
    obstacle = obstacle*100 + ones(4,2,2)*offset;

    iterations = 9000 ;
    q_start.coord = origin;
    q_start.input = 0;
    q_start.cost = 0;
    q_start.parent = origin;
    q_goal.cost = 0;
    q_goal.coord = goal;
    q_goal.input = 0;

    nodes(1) = q_start;
    tic
   
    %% Algorithm
    
     for i = 1:iterations
         % Pick a random point
         q_new.coord = random_point(width,height);
%          rand_cord = q_new.coord
         % Find the neareast node
         [q_nearest,q_new] = v_nearest(q_new,nodes);
         % Steer using dynamics constriants
         q_new = steer(q_new,q_nearest);
%          dynamics_cord = q_new.coord
         % Collision Check
         if collision_check(q_new.coord,q_nearest.coord,obstacle) && distance_euc(q_new.coord,q_nearest.coord)< 1000
             q_new.parent = q_nearest.coord;
             q_min = q_nearest;
             % Find nearby nodes
%              near_nodes = nearby (nodes , q_new ,obstacle);
%              % Revise point based on minimal cost of the nearby nodes
%              [q_new,q_min] = revise_cost(near_nodes,q_new,obstacle,q_min);
%              q_new.parent = q_min.coord;
             nodes = [nodes q_new];
             % Finding nearby nodes that benifit from rewiring 
%              nodes = rewire(near_nodes, q_new, obstacle , nodes , q_min);
         end
         if(goal_prox(q_new,q_goal))
             path = goal_path(nodes,q_goal);
             break
         end 
     end
     toc
%% Plot code

     plot_dynamics_rrt(nodes,obstacle,origin,path)

%      for i = 1:length(nodes)
%      vertex(i,:) = nodes(i).coord;
%      parent(i,:) = nodes(i).parent;
%      edges.x(i,:) = [vertex(i,1),parent(i,1)];
%      edges.y(i,:) = [vertex(i,2),parent(i,2)];
%      end
%      
%     figure('name', 'RRT basic');
%     scatter(origin(1), origin(2), 45, '*','r','LineWidth',1); hold on;
%     scatter(vertex(:,1), vertex(:,2), 10,linspace(1,10,length(vertex(:,1))),'filled'); hold on;
%     plot(edges.x', edges.y');
%     plot(obstacle(:,:,1),obstacle(:,:,2))

end

function prox = goal_prox (q_new,q_goal)
prox = 0;
dist = distance_euc(q_new.coord,q_goal.coord);
if (dist < 10)
    prox = 1;
end
end

%% need to implement a better way to search the shortest path using a very naive approach right now
function path = goal_path(nodes,q_goal)
    q_goal.parent = nodes(length(nodes)).coord;
    path = q_goal;
    q_next = q_goal;
    q_next.coord(1:2)
    nodes(1).coord(1:2)
    while(q_next.coord(1) ~= nodes(1).coord(1) || q_next.coord(2) ~= nodes(1).coord(2) )
       for i = 1:length(nodes)
           
           if nodes(i).coord == path(1).parent
               q_next = nodes(i);
               break
           end
       end
       path = [q_next path]; 
    end
    
end

function [q_new,q_min] = revise_cost(near_nodes,q_new,obstacle,q_min)
 for i = 1:length(near_nodes);
         new_cost = near_nodes(i).cost + distance_euc(q_new.coord,near_nodes(i).coord);
         if new_cost < q_new.cost
            q_min = near_nodes(i);
            q_new.cost = new_cost;
         end
 end
end

function near_nodes = nearby (nodes , q_new,obstacle)
near_nodes = [];
r = 1;
 for i = 1 : length(nodes)
%      if nodes(i).coord == q_new.parent; continue; end
     dist = distance_euc(q_new.coord, nodes(i).coord);
     if dist < r && collision_check(q_new.coord,nodes(i).coord,obstacle)
         near_nodes = [near_nodes nodes(i)];
     end
 end
end

function nodes = rewire(near_nodes, q_new, obstacle , nodes , q_min)
     for i = 1:length(near_nodes); 
         if near_nodes(i).coord == q_min.coord; continue ; end;
         temp_cost = (q_new.cost + distance_euc(q_new.coord , near_nodes(i).coord));
         if near_nodes(i).cost > temp_cost;
             if collision_check(q_new.coord,near_nodes(i).coord,obstacle)
                 near_nodes(i).parent = q_new.coord;
                 near_nodes(i).cost = temp_cost;
             end
         end
     end
 % Modifying the new nearby nodes in the actual node list
    for i = 1:length(near_nodes)
        if (near_nodes(i).parent == q_new.coord) & (near_nodes(i).coord ~= q_min.coord)
            for j = 1:length(nodes)
                if nodes(j).coord == near_nodes(i).coord %& collision_check(q_new.coord,near_nodes(i).coord,obstacle)
                    nodes(j).parent = near_nodes(i).parent;
                    nodes(j).cost = near_nodes(i).cost;
                end
            end
        end
    end
end

function [p_rand] = random_point(width,height)
 offset = [0,0] - [width, height]./2;
 x_rand = width*rand()+offset(1);
 y_rand = height*rand()+offset(2);
 theta_rand = 2*pi*rand();
 p_rand = [x_rand,y_rand,theta_rand,0,0];
end

 function [q_nearest,q_new] = v_nearest(q_new,nodes)
 p_rand = q_new.coord;
 q_new.cost = sqrt((p_rand(1,1)-nodes(1).coord(1,1))^2+(p_rand(1,2)-nodes(1).coord(1,2))^2);
 q_nearest.coord = nodes(1).coord;
 cost_near = nodes(1).cost;
 n = size(nodes);
 for i = 1:n(2)
   v  = nodes(i);
   new_dist = sqrt((p_rand(1,1)-v.coord(1))^2+(p_rand(1,2)-v.coord(2))^2);
   if new_dist< q_new.cost
       q_new.cost = new_dist;
       cost_near = v.cost;
       q_nearest.coord = v.coord;
   end
 end
 q_new.cost = q_new.cost + cost_near ;
 % Now find the nearest point according to local cost
 end 
  function d = distance_euc(x1,x2)
 d = sqrt((x1(1)-x2(1))^2+(x1(2)-x2(2))^2);
 
 end
 
 
 
 
 
 

