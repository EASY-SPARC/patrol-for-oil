function [robots, heading] = reactive_patrol(grid, robots, heading, mask, dist_grid, weights)

    n_robots = size(robots, 1);     % Number of robots
    aux_mask = mask;

    for robot = 1:n_robots
        neighbors = robots(setdiff(1:end, robot), :);
        target = computeTargetMulti(robots(robot, :), heading(robot), grid, neighbors, dist_grid,robot, weights(robot,:));
        if norm(target - robots(robot, :)) > 0
            % A*
            GoalRegister = int8(zeros(size(mask)));
            GoalRegister(target(2), target(1)) = 1;
            for k = 1:size(neighbors, 1)
               aux_mask(neighbors(k, 2), neighbors(k, 1)) = 1;
            end
            result = ASTARPATH(robots(robot, 1), robots(robot, 2), aux_mask, GoalRegister, 1);
            if size(result, 1) > 1
                move = [result(end - 1, 2) - robots(robot, 1), result(end - 1, 1) - robots(robot, 2)];

                robots(robot, :) = robots(robot, :) + move;
                heading(robot) = atan2(move(2), move(1));
            end
        else 
            disp(['Robot ', num2str(robot), ' stopped']);
        end
    end
end

%%
function target = computeTargetMulti(pos, heading, grid, neighbors, dist_grid,robot,weights)
    max_value = 0;
    target = pos;
    %max_heading = heading + pi;
    max_heading = 0;
    
    omega_c = weights(1);
    omega_s = weights(2);
    omega_d = weights(3);
    omega_n = weights(4);
    kappa = weights(5);
    
    for i = 1:size(grid, 2)
        for j = 1:size(grid, 1)
            if grid(j, i) < 0 % out of border conditions
                continue;
            else
                current = [i, j];
                if any(current ~= pos)
                    move = current - pos;
                    distance = norm(move);
                    new_heading = atan2(move(2), move(1));

                    distance_nearest_neigh = Inf;
                    for k = 1:size(neighbors, 1)
                        distance_neigh = norm(current - neighbors(k, :));
                        if (distance_neigh < distance_nearest_neigh)
                            distance_nearest_neigh = distance_neigh;
                        end
                    end
                    
                    if grid(j, i)>0
                        mapvalue(j,i) =max(kappa + omega_c*grid(j, i) + omega_s*dist_grid(j ,i) - omega_d * distance + omega_n * distance_nearest_neigh, 0);
                    else
                        mapvalue(j,i)=0;
                    end

                    value = mapvalue(j,i);
                    if (value > max_value) || (value == max_value && abs(new_heading - heading) <= abs(max_heading - heading))
                        target = current;
                        max_value = value;
                        max_heading = new_heading;
                    end
                end
            end
        end
    end
    figure(1)
    subplot(7,1,robot)
    mesh(mapvalue)
    if (max_value == 0)
        target = pos;
    end
end