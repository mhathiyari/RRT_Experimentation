function plot_dynamics_rrt(nodes,obstacle,origin,path)
     for i = 1:length(nodes)
         vertex(i,:) = nodes(i).coord(1:2);
         parent(i,:) = nodes(i).parent;
         input(i,:) = nodes(i).input;
         edges.x(i,:) = [vertex(i,1),parent(i,1)];
         edges.y(i,:) = [vertex(i,2),parent(i,2)];
     end
     
    figure('name', 'RRT basic');
    scatter(origin(1), origin(2), 45, '*','r','LineWidth',1); hold on;
    scatter(vertex(:,1), vertex(:,2), 10,linspace(1,10,length(vertex(:,1))),'filled');hold on;
    for i = 2:length(nodes)
        dest_point = nodes(i).coord;
        points = nodes(i).parent;
        q_nearest = nodes(i);
        q_nearest.coord = nodes(i).parent;
        [q_f,point_list] = new_state(q_nearest,nodes(i).input);
        if (point_list(end,:) ~= dest_point)
            disp('Error point mismatch');
            point_list(end,:)
            dest_point
        end
        points = [points;point_list];
        plot(points(:,1),points(:,2)); hold on;
    end
%     plot(edges.x', edges.y');
    scatter(path(:).coord(1), path(:).coord(2), 45, '*','b','LineWidth',1); hold on;
    plot(obstacle(:,:,1),obstacle(:,:,2))


end

