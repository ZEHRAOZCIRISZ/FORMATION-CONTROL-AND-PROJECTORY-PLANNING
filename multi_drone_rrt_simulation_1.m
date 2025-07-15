function multi_drone_rrt_simulation_1

    clc, clear;

    % Formation parameters
    d = 0.8; % Distance between drones in formation
    alpha = 3*pi/4; % Angle of V-formation (135 degrees)
    kf = 0.5; % Formation control gain

    % Parameters
    start_positions = [-4, -4, 0; -4, -3, 0; -3, -4, 0; -3, -3, 0; -2,-3,0]; % Starting positions for five drones
    goal_positions = [4, 4, 10; 4, 4, 10; 4, 4, 10; 4, 4, 10; 4, 4, 10]; % Goal positions for five drones
    step_size = 0.13; % Step size for RRT* (reduced from 0.5)
    max_iter = 1000; % Maximum iterations for RRT*
    goal_tolerance = 0.5; % Goal tolerance
    obstacle_speed = 0.5; % Speed of moving obstacles
    num_obstacles = 3; % Number of obstacles
    filename = 'multi_drone_path.gif'; % Output GIF file name
    collision_threshold = 2.0; % Minimum distance to consider as collision

    % Create figure
    figure('Color', 'w', 'Renderer', 'opengl');
    axis equal
    hold on
    grid on
    view(3)
    xlim([-5 5])
    ylim([-5 5])
    zlim([0 10])

    % Plot start and goal positions
    scatter3(start_positions(:,1), start_positions(:,2), start_positions(:,3), 'g', 'filled')
    scatter3(goal_positions(:,1), goal_positions(:,2), goal_positions(:,3), 'r', 'filled')

    % Initialize obstacles
    obstacles = initialize_obstacles(num_obstacles, [-5, 5], [-5, 5], [0, 10]);
    obstacle_plots = plot_obstacles(obstacles);

    % Perform RRT* path planning only for the leader (third drone)
    leader_path = rrt_star(start_positions(3,:), goal_positions(3,:), step_size, max_iter, goal_tolerance, obstacles, obstacle_speed);
    
    % Calculate follower paths based on leader path
    paths = cell(5, 1);
    paths{3} = leader_path;
    
    % Calculate formation positions for followers
    n = 5; % Number of drones
    l = 3; % Leader index (3rd drone)
    
    % Calculate follower paths
    for i = 1:n
        if i ~= l && ~isempty(leader_path)
            % Calculate desired distance from leader
            dis = d * abs(l-i);
            
            % Calculate formation angle based on position relative to leader
            if i < l
                ang = alpha; % Right side of V
            else
                ang = -alpha; % Left side of V
            end
            
            % Calculate desired position for each point in the path
            follower_path = zeros(size(leader_path));
            for j = 1:size(leader_path, 1)
                % Calculate leader's heading (direction of movement)
                if j < size(leader_path, 1)
                    leader_heading = atan2(leader_path(j+1,2) - leader_path(j,2), ...
                                        leader_path(j+1,1) - leader_path(j,1));
                else
                    leader_heading = atan2(leader_path(j,2) - leader_path(j-1,2), ...
                                        leader_path(j,1) - leader_path(j-1,1));
                end
                
                % Calculate follower's desired position
                pd = leader_path(j,:) + dis * [cos(leader_heading + ang), ...
                                             sin(leader_heading + ang), ...
                                             0];
                
                % Apply formation control
                follower_path(j,:) = pd;
            end
            paths{i} = follower_path;
        end
    end

    % Plot only the leader's (3rd drone) path
    if ~isempty(paths{3})
        plot3(paths{3}(:,1), paths{3}(:,2), paths{3}(:,3), 'k--', 'LineWidth', 1);
    end

    % Create drone models
    drones = create_drone_model();
    drone_plots = gobjects(5, 1);
    for k = 1:5
        drone_plots(k) = plot3(drones(:, 1) + start_positions(k, 1), ...
                               drones(:, 2) + start_positions(k, 2), ...
                               drones(:, 3) + start_positions(k, 3), 'k', 'LineWidth', 2);
    end

    % Initialize current positions and paused state
    current_positions = start_positions;
    is_paused = false;
    pause_counter = 0;
    max_pause_steps = 20; % Number of steps to wait when paused

    % Move the drones along their paths and save to GIF
    max_path_length = max(cellfun(@(p) size(p, 1), paths));
    i = 1;
    while i <= max_path_length
        % Update obstacles
        obstacles = update_obstacles(obstacles, [-5, 5], [-5, 5], [0, 10], obstacle_speed);

        % Update obstacle plots
        for j = 1:length(obstacles)
            switch obstacles(j).shape
                case 'sphere'
                    [x, y, z] = sphere;
                    x = x * obstacles(j).size + obstacles(j).pos(1);
                    y = y * obstacles(j).size + obstacles(j).pos(2);
                    z = z * obstacles(j).size + obstacles(j).pos(3);
                    set(obstacle_plots(j), 'XData', x, 'YData', y, 'ZData', z);
                case 'cube'
                    [x, y, z] = cube(obstacles(j).size);
                    vertices = [x(:), y(:), z(:)] + obstacles(j).pos;
                    set(obstacle_plots(j), 'Vertices', vertices);
                case 'cylinder'
                    [x, y, z] = cylinder(obstacles(j).size / 2);
                    x = x + obstacles(j).pos(1);
                    y = y + obstacles(j).pos(2);
                    z = z * obstacles(j).size + obstacles(j).pos(3);
                    set(obstacle_plots(j), 'XData', x, 'YData', y, 'ZData', z);
            end
        end

        % Check for collisions
        collision_detected = false;
        for k = 1:n
            if i <= size(paths{k}, 1)
                next_position = paths{k}(i,:);
                for j = 1:length(obstacles)
                    if norm(next_position - obstacles(j).pos) < collision_threshold
                        collision_detected = true;
                        break;
                    end
                end
            end
            if collision_detected
                break;
            end
        end

        % Handle collision or pause state
        if collision_detected || is_paused
            if ~is_paused
                is_paused = true;
                pause_counter = 0;
            end
            pause_counter = pause_counter + 1;
            
            if pause_counter >= max_pause_steps
                is_paused = false;
                pause_counter = 0;
            end
        else
            % Update drone positions if no collision and not paused
            for k = 1:n
                if i <= size(paths{k}, 1)
                    current_positions(k,:) = paths{k}(i,:);
                    set(drone_plots(k), 'XData', drones(:, 1) + current_positions(k, 1), ...
                                        'YData', drones(:, 2) + current_positions(k, 2), ...
                                        'ZData', drones(:, 3) + current_positions(k, 3));
                end
            end
            i = i + 1;
        end

        drawnow;

        % Capture the plot as an image
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        % Write to the GIF file
        if i == 1
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.2);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
        end
    end
end

function obstacles = initialize_obstacles(num_obstacles, xlim, ylim, zlim)
    obstacles = struct('pos', [], 'vel', [], 'size', [], 'shape', [], 'pattern', []);
    for i = 1:num_obstacles
        obstacles(i).pos = [rand * (xlim(2) - xlim(1)) + xlim(1), ...
                            rand * (ylim(2) - ylim(1)) + ylim(1), ...
                            rand * (zlim(2) - zlim(1)) + zlim(1)];
        obstacles(i).vel = rand(1, 3) * 0.2 - 0.1; % Random velocity
        base_size = rand * 1 + 0.5; % Base size for obstacles
        if mod(i, 3) == 0
            obstacles(i).shape = 'sphere';
            obstacles(i).size = base_size / 1.3; 
        elseif mod(i, 3) == 1
            obstacles(i).shape = 'cube' ;
            obstacles(i).size = base_size / 1.3;
        else
            obstacles(i).shape = 'cylinder';
            obstacles(i).size = base_size * 1.3;
        end
        % Define a movement pattern
        if mod(i, 2) == 0
            obstacles(i).pattern = 'circular';
        else
            obstacles(i).pattern = 'linear';
        end
    end
end

function obstacles = update_obstacles(obstacles, xlim, ylim, zlim, obstacle_speed)
    t = now * 100000; % Use current time as a variable to create smooth movement patterns
    for i = 1:length(obstacles)
        if strcmp(obstacles(i).pattern, 'circular')
            theta = obstacle_speed * t;
            obstacles(i).pos(1) = obstacles(i).pos(1) + cos(theta) * 0.1;
            obstacles(i).pos(2) = obstacles(i).pos(2) + sin(theta) * 0.1;
        else
            obstacles(i).pos = obstacles(i).pos + obstacles(i).vel * obstacle_speed;
            if obstacles(i).pos(1) < xlim(1) || obstacles(i).pos(1) > xlim(2)
                obstacles(i).vel(1) = -obstacles(i).vel(1);
            end
            if obstacles(i).pos(2) < ylim(1) || obstacles(i).pos(2) > ylim(2)
                obstacles(i).vel(2) = -obstacles(i).vel(2);
            end
            if obstacles(i).pos(3) < zlim(1) || obstacles(i).pos(3) > zlim(2)
                obstacles(i).vel(3) = -obstacles(i).vel(3);
            end
        end
    end
end

function path = rrt_star(start_pos, goal_pos, step_size, max_iter, goal_tolerance, obstacles, obstacle_speed)
    % RRT* Algorithm
    nodes = start_pos;
    parent = 0;
    goal_reached = false;

    for i = 1:max_iter
        % Update obstacles' positions
        obstacles = update_obstacles(obstacles, [-5, 5], [-5, 5], [0, 10], obstacle_speed);

        % Generate a random sample
        if rand < 0.1
            sample = goal_pos; % Bias towards the goal
        else
            sample = [rand*10-5, rand*10-5, rand*10]; % Random sample in space
        end

        % Find the nearest node
        distances = sqrt(sum((nodes - sample).^2, 2));
        [~, nearest_idx] = min(distances);
        nearest_node = nodes(nearest_idx, :);

        % Create a new node in the direction of the sample
        direction = (sample - nearest_node) / norm(sample - nearest_node);
        new_node = nearest_node + step_size * direction;

        % Check for collisions with obstacles
        if ~check_collision(new_node, obstacles)
            continue;
        end

        % Check if the new node is within the goal tolerance
        if norm(new_node - goal_pos) < goal_tolerance
            goal_reached = true;
            parent = [parent; nearest_idx];
            nodes = [nodes; goal_pos];
            break;
        end

        % Add the new node to the tree
        parent = [parent; nearest_idx];
        nodes = [nodes; new_node];
    end

    if goal_reached
        % Extract the path
        path = [goal_pos];
        node_idx = size(nodes, 1);
        while node_idx ~= 1
            node_idx = parent(node_idx);
            path = [nodes(node_idx, :); path];
        end
    else
        path = [];
    end
end

function collision = check_collision(point, obstacles)
    collision = true;
    for i = 1:length(obstacles)
        if norm(point - obstacles(i).pos) < (obstacles(i).size / 2)
            collision = false;
            return;
        end
    end
end

function drones = create_drone_model()
    % Create a more realistic quadcopter model
    arm_length = 0.5;
    body_width = 0.1;
    drones = [
        -arm_length, 0, 0;
         arm_length, 0, 0;
         NaN, NaN, NaN; % NaN to break the line
         0, -arm_length, 0;
         0, arm_length, 0;
         NaN, NaN, NaN; % NaN to break the line
         -body_width, body_width, 0;
         body_width, body_width, 0;
         body_width, -body_width, 0;
         -body_width, -body_width, 0;
         -body_width, body_width, 0;
    ];
end

function obstacle_plots = plot_obstacles(obstacles)
    obstacle_plots = gobjects(1, length(obstacles));
    for i = 1:length(obstacles)
        switch obstacles(i).shape
            case 'sphere'
                [x, y, z] = sphere;
                x = x * obstacles(i).size + obstacles(i).pos(1);
                y = y * obstacles(i).size + obstacles(i).pos(2);
                z = z * obstacles(i).size + obstacles(i).pos(3);
                obstacle_plots(i) = surf(x, y, z, 'FaceColor', 'r', 'EdgeColor', 'none');
            case 'cube'
                [x, y, z] = cube(obstacles(i).size);
                vertices = [x(:), y(:), z(:)] + obstacles(i).pos;
                obstacle_plots(i) = patch('Vertices', vertices, ...
                    'Faces', [1,2,6,5; 2,3,7,6; 3,4,8,7; 1,4,8,5; 1,2,3,4; 5,6,7,8], ...
                    'FaceColor', 'r', 'EdgeColor', 'none');
            case 'cylinder'
                [x, y, z] = cylinder(obstacles(i).size / 2);
                x = x + obstacles(i).pos(1);
                y = y + obstacles(i).pos(2);
                z = z * obstacles(i).size + obstacles(i).pos(3);
                obstacle_plots(i) = surf(x, y, z, 'FaceColor', 'r', 'EdgeColor', 'none');
        end
    end
end

function [x, y, z] = cube(size)
    x = [-1, 1, 1, -1, -1, 1, 1, -1] * size / 2;
    y = [-1, -1, 1, 1, -1, -1, 1, 1] * size / 2;
    z = [-1, -1, -1, -1, 1, 1, 1, 1] * size / 2;
end