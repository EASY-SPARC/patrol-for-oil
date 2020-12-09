function [robots, heading] = reactive_patrol(grid, robots, heading, mask)

    n_robots = size(robots, 1);     % Number of robots
    %robot_velocity = 90;           % Average robot velocity (km/h)
    omega_0 = -0.01;                % Repulsion to distant cells
    omega_1 = 0.1;                  % Repulsion to cells with nearby robots

    for robot = 1:n_robots
        target = computeTargetMulti(robots(robot, :), heading(robot), omega_0, omega_1, grid, robots(setdiff(1:end, robot), :));
        if norm(target - robots(robot, :)) > 0
            move = round((target - robots(robot, :)) ./ norm(target - robots(robot, :)));
            % Move robot, taking care of out of border
            if grid(robots(robot, 2) + move(2), robots(robot, 1) + move(1)) <= 0
                disp(['Robot ', num2str(robot), ' was going to an out-of-border cell']);
                % A*
                GoalRegister=int8(zeros(size(mask)));
                GoalRegister(target(2),target(1))=1;
                path = ASTARPATH(robots(robot, 2), robots(robot, 1), mask, GoalRegister,1);
                move = path(end - 1, :) - robots(robot, 1);
            end

            robots(robot, :) = robots(robot, :) + move;
            heading(robot) = atan2(move(2), move(1));
        else 
            disp(['Robot ', num2str(robot), ' stopped']);
        end
    end
end

%%
function target = computeTargetMulti(pos, heading, omega_0, omega_1, grid, neigh)
    max_value = 0;
    target = pos;
    %max_heading = heading + pi;
    max_heading = 0;
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
                    for k = 1:size(neigh, 1)
                        distance_neigh = norm(current - neigh(k, :));
                        if (distance_neigh < distance_nearest_neigh)
                            distance_nearest_neigh = distance_neigh;
                        end
                    end

                    value = max(grid(j, i) + omega_0 * distance + omega_1 * distance_nearest_neigh, 0);
                    if (value > max_value) || (value == max_value && abs(new_heading - heading) <= abs(max_heading - heading))
                        target = current;
                        max_value = value;
                        max_heading = new_heading;
                    end
                end
            end
        end
    end
    if (max_value == 0)
        target = pos;
    end
end