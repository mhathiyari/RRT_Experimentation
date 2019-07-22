function RRT_Star_musti()
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

    iterations = 400 ;
    vertecies = origin; 
    q_start.coord = origin;
    q_start.cost = 0;
    q_start.parent = origin;
    q_goal.cost = 0;
    q.goal.coord = goal;
    nodes(1) = q_start;
    
   
    %% Algorithm
    
     for i = 1:iterations
         % Pick a random point
         q_new.coord = random_point(width,height);
         % Find the neareast node
         [q_nearest,q_new] = v_nearest(q_new,nodes);
         % Steer using dynamics constriants
         q_new = steer(q_new,q_nearest);
         % Collision Check
         if collision_check(q_new.coord,q_nearest.coord,obstacle) && distance(q_new.coord,q_nearest.coord)< 3
             q_new.parent = q_nearest.coord;
             q_min = q_nearest;
             % Find nearby nodes
             near_nodes = nearby (nodes , q_new ,obstacle);
             % Revise point based on minimal cost of the nearby nodes
             [q_new,q_min] = revise_cost(near_nodes,q_new,obstacle,q_min);
             q_new.parent = q_min.coord;
             nodes = [nodes q_new];
             % Finding nearby nodes that benifit from rewiring 
             nodes = rewire(near_nodes, q_new, obstacle , nodes , q_min);
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

function [q_new,q_min] = revise_cost(near_nodes,q_new,obstacle,q_min)
 for i = 1:length(near_nodes);
         new_cost = near_nodes(i).cost + distance(q_new.coord,near_nodes(i).coord);
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
     dist = distance(q_new.coord, nodes(i).coord);
     if dist < r && collision_check(q_new.coord,nodes(i).coord,obstacle)
         near_nodes = [near_nodes nodes(i)];
     end
 end
end

function nodes = rewire(near_nodes, q_new, obstacle , nodes , q_min)
     for i = 1:length(near_nodes); 
         if near_nodes(i).coord == q_min.coord; continue ; end;
         temp_cost = (q_new.cost + distance(q_new.coord , near_nodes(i).coord));
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
 p_rand = [x_rand,y_rand];
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
 
 function q_new = steer(q_new,q_nearest)
    q_new.coord = round(q_new.coord,3);
    q_new.cost = q_new.cost;
    
 end
 
 
 
 
 
 
 
 

