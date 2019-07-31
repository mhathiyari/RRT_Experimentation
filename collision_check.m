 % taken from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 function collide = collision_check(p_new,p_nearest,obstacle) %has to be a better way to do this 
%  collide = 1; 
%  return   
    for i = 1:length(obstacle)
         p1 = [obstacle(i,1,1),obstacle(i,1,2)];
         q1 = [obstacle(i,2,1),obstacle(i,2,2)];
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
 