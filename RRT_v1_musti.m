function RRT_v1_musti()
    width = 10;
    height = 10;

    origin = [5,2];
    goal = [8,8];
    % box
    obstacle = zeros(4,2,2);
    obstacle(1,:,:) = [1,1;-1,1];
    obstacle(2,:,:) = [-1,1;-1,-1];
    obstacle(3,:,:) = [-1,-1;1,-1];
    obstacle(4,:,:) = [1,-1;1,1];
    offset = 0;
    obstacle = obstacle + ones(4,2,2)*offset;

    iterations = 1000 ;
    vertecies = origin; 
    edges.x = zeros(iterations,2);
    edges.y = zeros(iterations,2);
    ind_nearest = zeros(iterations,1);
    vert_count = 1;
    edge_count = 0
     for i = 1:iterations
         p_rand = random_point(width,height);
         p_nearest = v_nearest(p_rand,vertecies);
         p_new = steer(p_rand,p_nearest);
         if collision_check(p_new,p_nearest,obstacle)
             vert_count = vert_count + 1;
             vertecies(vert_count,:) = p_new;
             edge_count = edge_count + 1;
             edges.x(edge_count,:) = [p_nearest(1),p_new(1)];
             edges.y(edge_count,:) = [p_nearest(2),p_new(2)];
         end
     end
    figure('name', 'RRT basic');
    scatter(origin(1), origin(2), 45, '*','r','LineWidth',1); hold on;
    scatter(vertecies(:,1), vertecies(:,2), 10,linspace(1,10,length(vertecies(:,1))),'filled'); hold on;
    plot(edges.x', edges.y');
    plot(obstacle(:,:,1),obstacle(:,:,2))

end


function p_rand = random_point(width,height)
 offset = [0,0] - [width, height]./2;
 x_rand = width*rand()+offset(1);
 y_rand = height*rand()+offset(2);
 p_rand = [x_rand,y_rand];
end

 function p_nearest = v_nearest(p_rand,vertecies)
 dist = sqrt((p_rand(1,1)-vertecies(1,1))^2+(p_rand(1,2)-vertecies(1,2))^2);
 p_nearest = vertecies(1,:);
 n = size(vertecies);
 for i = 1:n(1)
   new_dist = sqrt((p_rand(1,1)-vertecies(i,1))^2+(p_rand(1,2)-vertecies(i,2))^2);
   if new_dist< dist
       dist = new_dist;
       p_nearest = vertecies(i,:);
   end
 end
 end
 
 function p_new = steer(p_rand,p_nearest)
    p_new = round(p_rand,3);
 end
 % taken from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 function collide = collision_check(p_new,p_nearest,obstacle) %has to be a better way to do this 
%  collide = 1; 
%  return   
    for i = 1:length(obstacle)
         p1 = [obstacle(i,1,1),obstacle(i,2,1)];
         q1 = [obstacle(i,1,2),obstacle(i,2,2)];
         o1 = orientation(p_nearest,p1,p_new);
         o2 = orientation(p_nearest,q1,p_new);
         o3 = orientation(p1,p_nearest,q1);     
         o4 = orientation(p1,p_new,q1);

        if (o1 ~= o2 && o3 ~= o4)
            collide = 0;
            return;
        end
         collide = 0;
         if (o1 == 0 && onsegment(p_nearest,p_new,p1))
             return;
         end
        if (o2 == 0 && onsegment(p_nearest,p_new,q1))
         return;
        end
        if (o3 == 0 && onsegment(p1,q1,p_nearest))
         return;
        end
        if (o4 == 0 && onsegment(p1,q1,p_new))
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
 
