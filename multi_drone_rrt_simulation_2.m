function multi_drone_rrt_simulation_2

    clc, clear;

    % Formation parameters
    d = 0.8; % Distance between drones in formation
    alpha = 3*pi/4; % Angle of V-formation (135 degrees)
    kf = 0.5; % Formation control gain

    % Parameters
    start_positions = [-4, -4, 0; -4, -3, 0; -1, -4, 0; -3, -3, 0; -2,-3,0]; % Starting positions for five drones
    goal_positions = [4, 4, 10; 4, 4, 10; 4, 4, 10; 4, 4, 10; 4, 4, 10]; % Goal positions for five drones
    step_size = 0.10; % Step size for RRT*
    max_iter = 1000; % Maximum iterations for RRT*
    goal_tolerance = 0.1; % Goal tolerance
    obstacle_speed = 0.6; % Speed of moving obstacles
    num_obstacles = 3; % Number of obstacles
    filename = 'multi_drone_path.gif'; % Output GIF file name
    collision_threshold = 2.0; % Minimum distance to consider as collision
    n = 5; % Number of drones
    l = (n+1)/2; % Leader index (3rd drone)

    % Create figure
    figure('Color', 'w', 'Renderer', 'opengl');
    axis equal
    hold on
    grid on
    view(3)
    xlim([-5 5])
    ylim([-5 5])
    zlim([0 10])

    % Add restart button
    restart_btn = uicontrol('Style', 'pushbutton', ...
                           'String', 'Restart Simulation', ...
                           'Position', [20 20 150 30], ...
                           'Callback', 'clf; multi_drone_rrt_simulation_2();');

    % --- Sayaçlar ---
    rrt_total_steps = 0;
    sim_start_time = tic;

    % Plot start and goal positions
    scatter3(start_positions(:,1), start_positions(:,2), start_positions(:,3), 'g', 'filled')
    scatter3(goal_positions(:,1), goal_positions(:,2), goal_positions(:,3), 'r', 'filled')

    % Initialize obstacles
    obstacles = initialize_obstacles(num_obstacles, [-5, 5], [-5, 5], [0, 10]);
    obstacle_plots = plot_obstacles(obstacles);

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

    % Formation initialization phase
    formation_steps = 50; % Reduced number of steps for formation building
    formation_positions = zeros(5, 3, formation_steps);
    frame_delay = 0.2; % Consistent frame delay for both phases
    
    % Initialize path planning variables
    paths = cell(5, 1);
    leader_path = [];
    path_planning_complete = false;
    
    % Calculate initial formation positions and perform path planning
    for step = 1:formation_steps
        t = step/formation_steps; % Normalized time [0,1]
        
        % Leader stays at its initial position
        leader_pos = start_positions(3,:);
        
        % Calculate follower positions
        for k = 1:5
            if k == 3 % Leader
                formation_positions(k,:,step) = leader_pos; % Leader stays stationary
            else
                % Calculate desired formation position
                dis = d * abs(3-k);
                if k < 3
                    ang = alpha;
                else
                    ang = -alpha;
                end
                
                % Calculate follower's target position relative to stationary leader
                target_pos = leader_pos + dis * [cos(ang), sin(ang), 0];
                
                % Interpolate between start and target position
                formation_positions(k,:,step) = start_positions(k,:) + t * (target_pos - start_positions(k,:));
            end
        end
        
        % Perform path planning during formation building
        if ~path_planning_complete && step > formation_steps/2
            % Perform RRT* path planning for the leader
            [leader_path, rrt_steps] = rrt_star(start_positions(3,:), goal_positions(3,:), step_size, max_iter, goal_tolerance, obstacles, obstacle_speed);
            rrt_total_steps = rrt_total_steps + rrt_steps;
            
            if ~isempty(leader_path)
                paths{3} = leader_path;
                
                % Calculate follower paths based on leader path
                for k = 1:5
                    if k ~= 3 && ~isempty(leader_path)
                        % Calculate desired distance from leader
                        dis = d * abs(3-k);
                        
                        % Calculate formation angle based on position relative to leader
                        if k < 3
                            ang = alpha;
                        else
                            ang = -alpha;
                        end
                        
                        % Calculate desired position for each point in the path
                        follower_path = zeros(size(leader_path));
                        for j = 1:size(leader_path, 1)
                            % Calculate leader's heading
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
                            
                            follower_path(j,:) = pd;
                        end
                        paths{k} = follower_path;
                    end
                end
                path_planning_complete = true;
            end
        end
        
        % Update drone positions for formation building
        for k = 1:5
            current_positions(k,:) = formation_positions(k,:,step);
            set(drone_plots(k), 'XData', drones(:, 1) + current_positions(k, 1), ...
                                'YData', drones(:, 2) + current_positions(k, 2), ...
                                'ZData', drones(:, 3) + current_positions(k, 3));
        end
        
        % Plot the planned path if available
        if path_planning_complete && ~isempty(paths{3})
            plot3(paths{3}(:,1), paths{3}(:,2), paths{3}(:,3), 'k--', 'LineWidth', 1);
        end
        
        drawnow;
        
        % Capture the plot as an image
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        
        % Write to the GIF file
        if step == 1
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', frame_delay);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', frame_delay);
        end
    end
    
    % Then continue with the original path following
    max_path_length = max(cellfun(@(p) size(p, 1), paths));
    i = 1;
    original_paths = paths; % Store original paths
    safety_distance = 2.0; % Safety distance around obstacles
    avoidance_mode = false;
    avoidance_start_idx = 0;
    avoidance_end_idx = 0;
    
    while i <= max_path_length
        % Formasyon kurma: Dronlar, yolun başında formasyona girsin
        if i <= formation_steps
            t = i/formation_steps;
            leader_pos = paths{3}(i,:);
            for k = 1:5
                if k == 3
                    current_positions(k,:) = leader_pos;
                else
                    dis = d * abs(3-k);
                    if k < 3
                        ang = alpha;
                    else
                        ang = -alpha;
                    end
                    target_pos = leader_pos + dis * [cos(ang), sin(ang), 0];
                    % Başlangıçtan formasyon pozisyonuna lineer geçiş
                    current_positions(k,:) = start_positions(k,:) + t * (target_pos - start_positions(k,:));
                end
                set(drone_plots(k), 'XData', drones(:, 1) + current_positions(k, 1), ...
                                    'YData', drones(:, 2) + current_positions(k, 2), ...
                                    'ZData', drones(:, 3) + current_positions(k, 3));
            end
        else
            % Formasyon tamamlandıktan sonra, yolları takip etsinler
            for k = 1:n
                if i <= size(paths{k}, 1)
                    current_positions(k,:) = paths{k}(i,:);
                    set(drone_plots(k), 'XData', drones(:, 1) + current_positions(k, 1), ...
                                        'YData', drones(:, 2) + current_positions(k, 2), ...
                                        'ZData', drones(:, 3) + current_positions(k, 3));
                end
            end
        end

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

        % Check for collisions and handle obstacle avoidance
        collision_detected = false;
        closest_obstacle = [];
        min_distance = inf;
        
        % Check collisions for each drone
        for k = 1:n
            if i <= size(paths{k}, 1)
                next_position = paths{k}(i,:);
                
                % Check collision with obstacles
                for j = 1:length(obstacles)
                    dist = norm(next_position - obstacles(j).pos);
                    if dist < collision_threshold
                        collision_detected = true;
                        if dist < min_distance
                            min_distance = dist;
                            closest_obstacle = obstacles(j);
                        end
                    end
                end
                
                % Check collision with other drones
                for other_drone = 1:n
                    if other_drone ~= k && i <= size(paths{other_drone}, 1)
                        other_position = paths{other_drone}(i,:);
                        drone_dist = norm(next_position - other_position);
                        if drone_dist < d % Minimum distance between drones
                            collision_detected = true;
                            if drone_dist < min_distance
                                min_distance = drone_dist;
                                closest_obstacle = struct('pos', other_position, 'size', d/2);
                            end
                        end
                    end
                end
            end
        end

        if collision_detected && ~avoidance_mode
            % Start avoidance mode
            avoidance_mode = true;
            avoidance_start_idx = i;
            
            % Calculate avoidance path for each drone
            for k = 1:n
                if i <= size(paths{k}, 1)
                    current_pos = current_positions(k,:);
                    next_pos = paths{k}(i,:);
                    
                    % Calculate vector from obstacle to current position
                    obs_to_pos = current_pos - closest_obstacle.pos;
                    obs_to_pos = obs_to_pos / norm(obs_to_pos);
                    
                    % Calculate perpendicular vector for avoidance
                    perp_vector = cross(obs_to_pos, [0, 0, 1]);
                    perp_vector = perp_vector / norm(perp_vector);
                    
                    % Calculate avoidance point with increased safety distance for drone-drone collisions
                    if isfield(closest_obstacle, 'size') && closest_obstacle.size == d/2
                        safety_distance = d * 1.5; % Increased safety distance for drone-drone avoidance
                    else
                        safety_distance = 2.0; % Original safety distance for obstacle avoidance
                    end
                    
                    % Calculate avoidance point
                    avoidance_point = closest_obstacle.pos + (obs_to_pos + perp_vector) * safety_distance;
                    
                    % Create temporary path segment for avoidance
                    temp_path = [current_pos; avoidance_point; next_pos];
                    paths{k} = [paths{k}(1:i-1,:); temp_path; paths{k}(i+1:end,:)];
                end
            end
        elseif ~collision_detected && avoidance_mode
            % End avoidance mode and return to original path
            avoidance_mode = false;
            avoidance_end_idx = i;
            
            % Restore original paths
            for k = 1:n
                if ~isempty(original_paths{k})
                    paths{k} = original_paths{k};
                    i = avoidance_end_idx; % Continue from where we left off
                end
            end
        end

        % Update drone positions
        for k = 1:n
            if i <= size(paths{k}, 1)
                current_positions(k,:) = paths{k}(i,:);
                set(drone_plots(k), 'XData', drones(:, 1) + current_positions(k, 1), ...
                                    'YData', drones(:, 2) + current_positions(k, 2), ...
                                    'ZData', drones(:, 3) + current_positions(k, 3));
            end
        end
        i = i + 1;

        drawnow;

        % Capture the plot as an image
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        % Write to the GIF file
        if i == 1
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', frame_delay);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', frame_delay);
        end
    end

    % Simülasyonun sonunda süreyi ve RRT adım sayısını command window'da göster
    sim_elapsed_time = toc(sim_start_time);
    fprintf('Hedefe ulaşma süresi: %.2f sn\nRRT* toplam adım sayısı: %d\n', sim_elapsed_time, rrt_total_steps);

    % Add restart function at the end of the file
    function restartSimulation()
        % Clear the current figure
        clf;
        % Restart the simulation
        multi_drone_rrt_simulation_2();
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
            obstacles(i).size = base_size ;
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

function [path, step_count] = rrt_star(start_pos, goal_pos, step_size, max_iter, goal_tolerance, obstacles, obstacle_speed)
    % RRT* Algorithm
    nodes = start_pos;
    parent = 0;
    cost = 0;  % Cost from start to each node
    goal_reached = false;
    search_radius = 2 * step_size;  % Radius for finding neighbors
    step_count = 0;

    for i = 1:max_iter
        step_count = step_count + 1;
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

        % Find neighbors within search radius
        neighbor_distances = sqrt(sum((nodes - new_node).^2, 2));
        neighbors = find(neighbor_distances <= search_radius);

        % Find the best parent node
        min_cost = inf;
        best_parent = nearest_idx;
        
        for j = 1:length(neighbors)
            neighbor_idx = neighbors(j);
            if neighbor_idx == nearest_idx
                continue;
            end
            
            % Calculate cost through this neighbor
            potential_cost = cost(neighbor_idx) + norm(new_node - nodes(neighbor_idx,:));
            
            if potential_cost < min_cost
                min_cost = potential_cost;
                best_parent = neighbor_idx;
            end
        end

        % Add the new node to the tree
        parent = [parent; best_parent];
        nodes = [nodes; new_node];
        cost = [cost; min_cost];

        % --- Her yeni node için parent ile arasında siyah çizgi çiz ---
        parent_node = nodes(best_parent, :);
        plot3([parent_node(1) new_node(1)], [parent_node(2) new_node(2)], [parent_node(3) new_node(3)], 'k-', 'LineWidth', 1);
        drawnow limitrate;

        % Rewire the tree
        for j = 1:length(neighbors)
            neighbor_idx = neighbors(j);
            if neighbor_idx == best_parent
                continue;
            end
            
            % Calculate potential new cost for neighbor
            potential_cost = cost(end) + norm(nodes(neighbor_idx,:) - new_node);
            
            if potential_cost < cost(neighbor_idx)
                % Update parent and cost
                parent(neighbor_idx) = length(nodes);
                cost(neighbor_idx) = potential_cost;
            end
        end

        % Check if the new node is within the goal tolerance
        if norm(new_node - goal_pos) < goal_tolerance
            goal_reached = true;
            parent = [parent; length(nodes)];
            nodes = [nodes; goal_pos];
            cost = [cost; cost(end) + norm(goal_pos - new_node)];
            break;
        end
    end

    if goal_reached
        % Extract the path
        path = [goal_pos];
        node_idx = size(nodes, 1);
        while node_idx ~= 1
            node_idx = parent(node_idx);
            path = [nodes(node_idx, :); path];
        end
        % --- Yolun tamamını kırmızı çizgiyle çiz ---
        plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2);
        drawnow;
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