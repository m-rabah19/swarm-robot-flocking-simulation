% SWARM ROBOT FLOCKING SIMULATION
% Comprehensive Implementation of Reynolds' Boids Algorithm
% Robotics and Control Project
% Muhammed Rabah 22BEI0108
% Aafthab Ahmed 22BEE0357

function swarm_flocking_simulation()
    %% INITIALIZATION PHASE
    clear; close all; clc;
    
    % Simulation Parameters
    num_robots = 25;
    simulation_time = 100;
    dt = 0.05;
    area_size = 60;
    
    % Behavior Weights
    w_separation = 1.8;
    w_alignment = 1.2;
    w_cohesion = 1.1;
    w_obstacle = 2.5;
    w_goal = 0.8;
    w_noise = 0.15;
    
    % Physical Constraints
    max_speed = 2.5;
    max_force = 0.4;
    mass = 1.0;
    
    % Perception Radii
    R_separation = 8;
    R_alignment = 12;
    R_cohesion = 12;
    R_obstacle = 15;
    
    epsilon = 1e-6;
    
    fprintf('Initializing swarm with %d robots...\n', num_robots);
    
    % Initialize Robot Positions and Velocities
    robots_pos = area_size * (rand(num_robots, 2) - 0.5);
    robots_vel = 2 * (rand(num_robots, 2) - 0.5);
    
    % Normalize Initial Velocities
    speed = sqrt(sum(robots_vel.^2, 2));
    for i = 1:num_robots
        if speed(i) > 0
            robots_vel(i,:) = (robots_vel(i,:) / speed(i)) * max_speed;
        end
    end
    
    % Initialize Obstacles
    obstacles = [
        15, 15, 6;
        -12, -8, 5;
        10, -15, 4;
        -18, 10, 5
    ];
    
    % Initialize Goal
    goal_position = [25, 20];
    
    % Performance Metrics
    performance_metrics = struct();
    performance_metrics.formation_error = [];
    performance_metrics.alignment_quality = [];
    performance_metrics.min_safety = [];
    performance_metrics.energy_consumption = 0;
    
    %% VISUALIZATION SETUP
    figure('Position', [100, 100, 1200, 900]);
    axis equal;
    xlim([-area_size, area_size]);
    ylim([-area_size, area_size]);
    grid on;
    hold on;
    
    % Plot Obstacles
    theta = linspace(0, 2*pi, 100);
    for i = 1:size(obstacles, 1)
        x_obs = obstacles(i,1) + obstacles(i,3) * cos(theta);
        y_obs = obstacles(i,2) + obstacles(i,3) * sin(theta);
        fill(x_obs, y_obs, [0.7, 0.1, 0.1], 'FaceAlpha', 0.7, ...
             'EdgeColor', [0.5, 0, 0], 'LineWidth', 2);
    end
    
    % Plot Goal
    plot(goal_position(1), goal_position(2), 'gh', ...
         'MarkerSize', 20, 'MarkerFaceColor', [0, 0.9, 0], 'LineWidth', 3);
    
    % Initialize Visualization Handles
    h_robots = scatter(robots_pos(:,1), robots_pos(:,2), 100, ...
                       'filled', 'MarkerFaceColor', [0.2, 0.4, 0.9]);
    h_quiver = quiver(robots_pos(:,1), robots_pos(:,2), ...
                      robots_vel(:,1), robots_vel(:,2), 0, ...
                      'Color', [0.8, 0, 0.8], 'LineWidth', 2.5, ...
                      'MaxHeadSize', 1.0);
    h_goal_vector = quiver(robots_pos(:,1), robots_pos(:,2), ...
                           zeros(num_robots,1), zeros(num_robots,1), 0, ...
                           'Color', [0, 0.7, 0.7], 'LineWidth', 1.5, ...
                           'MaxHeadSize', 0.8);
    
    % Perception Radius Visualization
    theta_viz = linspace(0, 2*pi, 100);
    viz_robot = 1;
    x_viz = robots_pos(viz_robot,1) + R_alignment * cos(theta_viz);
    y_viz = robots_pos(viz_robot,2) + R_alignment * sin(theta_viz);
    h_perception = plot(x_viz, y_viz, '--', 'Color', [0.9, 0.6, 0.2], ...
                        'LineWidth', 1.5);
    
    title('Swarm Robot Flocking Simulation - Color Coded', ...
          'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X Position (m)', 'FontSize', 12);
    ylabel('Y Position (m)', 'FontSize', 12);
    
    legend([h_robots, h_quiver, h_goal_vector, h_perception], ...
           {'Robots', 'Velocity Vectors', 'Goal Direction', ...
            'Perception Radius'}, 'Location', 'northeastoutside', ...
           'FontSize', 10);
    
    %% MAIN SIMULATION LOOP
    fprintf('Starting simulation...\n');
    
    for t = 0:dt:simulation_time
        acceleration = zeros(num_robots, 2);
        total_energy_step = 0;
        
        % Calculate behaviors for each robot
        for i = 1:num_robots
            current_pos = robots_pos(i,:);
            current_vel = robots_vel(i,:);
            
            % Find neighbors
            neighbors_sep = find_neighbors(i, robots_pos, R_separation, epsilon);
            neighbors_align = find_neighbors(i, robots_pos, R_alignment, epsilon);
            neighbors_coh = find_neighbors(i, robots_pos, R_cohesion, epsilon);
            
            % Calculate forces
            F_sep = calculate_separation_force(i, robots_pos, neighbors_sep, ...
                                              w_separation, epsilon);
            F_align = calculate_alignment_force(i, robots_vel, neighbors_align, ...
                                               w_alignment);
            F_coh = calculate_cohesion_force(i, robots_pos, neighbors_coh, ...
                                            w_cohesion);
            F_obs = calculate_obstacle_force(current_pos, obstacles, ...
                                            R_obstacle, w_obstacle, epsilon);
            F_goal = calculate_goal_force(current_pos, goal_position, w_goal);
            F_noise = w_noise * (rand(1,2) - 0.5) * 2;
            
            % Combine forces
            F_total = F_sep + F_align + F_coh + F_obs + F_goal + F_noise;
            
            % Limit total force
            force_magnitude = norm(F_total);
            if force_magnitude > max_force
                F_total = (F_total / force_magnitude) * max_force;
            end
            
            acceleration(i,:) = F_total / mass;
            total_energy_step = total_energy_step + norm(current_vel)^2 + norm(F_total)^2;
        end
        
        % Update performance metrics
        performance_metrics.energy_consumption = performance_metrics.energy_consumption + total_energy_step * dt;
        performance_metrics.formation_error(end+1) = calculate_formation_error(robots_pos);
        performance_metrics.alignment_quality(end+1) = calculate_alignment_quality(robots_vel, robots_pos, R_alignment, epsilon);
        performance_metrics.min_safety(end+1) = calculate_min_safety(robots_pos);
        
        % Update velocities and positions
        robots_vel = robots_vel + acceleration * dt;
        
        % Limit speed
        speed = sqrt(sum(robots_vel.^2, 2));
        for i = 1:num_robots
            if speed(i) > max_speed
                robots_vel(i,:) = (robots_vel(i,:) / speed(i)) * max_speed;
            end
        end
        
        robots_pos = robots_pos + robots_vel * dt;
        
        % Apply boundary conditions
        robots_pos(robots_pos > area_size) = robots_pos(robots_pos > area_size) - 2*area_size;
        robots_pos(robots_pos < -area_size) = robots_pos(robots_pos < -area_size) + 2*area_size;
        
        % Update visualization
        update_visualization_colored(h_robots, h_quiver, h_goal_vector, ...
                                    h_perception, robots_pos, robots_vel, ...
                                    goal_position, R_alignment);
        
        % Update title with performance metrics
        if ~isempty(performance_metrics.formation_error)
            title(sprintf(['Swarm Robot Flocking Simulation - Color Coded\n' ...
                           'Time: %.1f s | Formation Error: %.2f | ' ...
                           'Alignment: %.2f | Safety: %.2f'], ...
                          t, performance_metrics.formation_error(end), ...
                          performance_metrics.alignment_quality(end), ...
                          performance_metrics.min_safety(end)), ...
                  'FontSize', 12, 'FontWeight', 'bold');
        end
        
        drawnow;
        pause(0.02);
    end
    
    %% PERFORMANCE ANALYSIS
    fprintf('\n=== SIMULATION COMPLETE ===\n');
    fprintf('Total Simulation Time: %.1f seconds\n', simulation_time);
    fprintf('Final Performance Metrics:\n');
    if ~isempty(performance_metrics.formation_error)
        fprintf('  - Average Formation Error: %.3f\n', mean(performance_metrics.formation_error));
        fprintf('  - Average Alignment Quality: %.3f\n', mean(performance_metrics.alignment_quality));
        fprintf('  - Minimum Safety Margin: %.3f\n', min(performance_metrics.min_safety));
        fprintf('  - Total Energy Consumption: %.3f\n', performance_metrics.energy_consumption);
    end
    
    generate_performance_plots(performance_metrics, dt, simulation_time, ...
                              robots_pos, robots_vel);
end

%% SUPPORTING FUNCTIONS

function neighbors = find_neighbors(robot_id, positions, radius, epsilon)
    current_pos = positions(robot_id,:);
    distances = sqrt(sum((positions - current_pos).^2, 2));
    neighbors = find(distances < radius & distances > epsilon);
end

function F_sep = calculate_separation_force(robot_id, positions, neighbors, w_sep, epsilon)
    current_pos = positions(robot_id,:);
    F_sep = [0, 0];
    
    if isempty(neighbors)
        return;
    end
    
    for j = 1:length(neighbors)
        neighbor_pos = positions(neighbors(j),:);
        diff = current_pos - neighbor_pos;
        distance_sq = sum(diff.^2) + epsilon;
        F_sep = F_sep + diff / distance_sq;
    end
    
    if norm(F_sep) > 0
        F_sep = (F_sep / norm(F_sep)) * w_sep;
    end
end

function F_align = calculate_alignment_force(robot_id, velocities, neighbors, w_align)
    current_vel = velocities(robot_id,:);
    F_align = [0, 0];
    
    if isempty(neighbors)
        return;
    end
    
    avg_velocity = mean(velocities(neighbors,:), 1);
    F_align = (avg_velocity - current_vel) * w_align;
end

function F_coh = calculate_cohesion_force(robot_id, positions, neighbors, w_coh)
    current_pos = positions(robot_id,:);
    F_coh = [0, 0];
    
    if isempty(neighbors)
        return;
    end
    
    center_of_mass = mean(positions(neighbors,:), 1);
    direction = center_of_mass - current_pos;
    
    if norm(direction) > 0
        F_coh = (direction / norm(direction)) * w_coh;
    end
end

function F_obs = calculate_obstacle_force(position, obstacles, R_obs, w_obs, epsilon)
    F_obs = [0, 0];
    
    for k = 1:size(obstacles, 1)
        obs_pos = obstacles(k, 1:2);
        obs_radius = obstacles(k, 3);
        distance = norm(position - obs_pos);
        
        if distance < (R_obs + obs_radius)
            diff = position - obs_pos;
            distance_sq = sum(diff.^2) + epsilon;
            F_obs = F_obs + diff / distance_sq;
        end
    end
    
    if norm(F_obs) > 0
        F_obs = (F_obs / norm(F_obs)) * w_obs;
    end
end

function F_goal = calculate_goal_force(position, goal, w_goal)
    direction = goal - position;
    
    if norm(direction) > 0
        F_goal = (direction / norm(direction)) * w_goal;
    else
        F_goal = [0, 0];
    end
end

function formation_error = calculate_formation_error(positions)
    centroid = mean(positions, 1);
    distances = sqrt(sum((positions - centroid).^2, 2));
    formation_error = mean(distances);
end

function alignment_quality = calculate_alignment_quality(velocities, positions, R_align, epsilon)
    total_alignment = 0;
    count = 0;
    
    for i = 1:size(positions, 1)
        neighbors = find_neighbors(i, positions, R_align, epsilon);
        if ~isempty(neighbors)
            for j = 1:length(neighbors)
                vi = velocities(i,:);
                vj = velocities(neighbors(j),:);
                vi_norm = norm(vi);
                vj_norm = norm(vj);
                if vi_norm > 0 && vj_norm > 0
                    correlation = dot(vi, vj) / (vi_norm * vj_norm);
                    total_alignment = total_alignment + correlation;
                    count = count + 1;
                end
            end
        end
    end
    
    if count > 0
        alignment_quality = total_alignment / count;
    else
        alignment_quality = 0;
    end
end

function min_safety = calculate_min_safety(positions)
    if size(positions, 1) < 2
        min_safety = inf;
        return;
    end
    
    min_distance = inf;
    for i = 1:size(positions, 1)
        for j = i+1:size(positions, 1)
            distance = norm(positions(i,:) - positions(j,:));
            if distance < min_distance
                min_distance = distance;
            end
        end
    end
    min_safety = min_distance;
end

function update_visualization_colored(h_robots, h_quiver, h_goal_vector, ...
                                     h_perception, positions, velocities, ...
                                     goal, R_align)
    % Update robots
    set(h_robots, 'XData', positions(:,1), 'YData', positions(:,2));
    
    % Update velocity vectors
    set(h_quiver, 'XData', positions(:,1), 'YData', positions(:,2), ...
                  'UData', velocities(:,1), 'VData', velocities(:,2));
    
    % Update goal vectors
    goal_vectors = zeros(size(positions));
    for i = 1:size(positions, 1)
        direction = goal - positions(i,:);
        if norm(direction) > 0
            goal_vectors(i,:) = direction / norm(direction) * 3;
        end
    end
    set(h_goal_vector, 'XData', positions(:,1), 'YData', positions(:,2), ...
                       'UData', goal_vectors(:,1), 'VData', goal_vectors(:,2));
    
    % Update perception radius
    theta_viz = linspace(0, 2*pi, 100);
    x_viz = positions(1,1) + R_align * cos(theta_viz);
    y_viz = positions(1,2) + R_align * sin(theta_viz);
    set(h_perception, 'XData', x_viz, 'YData', y_viz);
end

function generate_performance_plots(performance_metrics, dt, simulation_time, ...
                                   robots_pos, robots_vel)
    if ~isempty(performance_metrics.formation_error)
        figure('Position', [100, 100, 1400, 600]);
        
        % Formation Error Over Time
        subplot(2,3,1);
        time_vector = 0:dt:simulation_time;
        plot(time_vector(1:length(performance_metrics.formation_error)), ...
             performance_metrics.formation_error, 'b-', 'LineWidth', 2);
        title('Formation Error Over Time', 'FontSize', 12);
        xlabel('Time (s)');
        ylabel('Error');
        grid on;
        
        % Alignment Quality Over Time
        subplot(2,3,2);
        plot(time_vector(1:length(performance_metrics.alignment_quality)), ...
             performance_metrics.alignment_quality, 'm-', 'LineWidth', 2);
        title('Alignment Quality Over Time', 'FontSize', 12);
        xlabel('Time (s)');
        ylabel('Alignment');
        grid on;
        
        % Safety Margin Over Time
        subplot(2,3,3);
        plot(time_vector(1:length(performance_metrics.min_safety)), ...
             performance_metrics.min_safety, 'r-', 'LineWidth', 2);
        title('Safety Margin Over Time', 'FontSize', 12);
        xlabel('Time (s)');
        ylabel('Minimum Distance');
        grid on;
        
        % Energy Consumption Rate
        subplot(2,3,4);
        power = (performance_metrics.energy_consumption / simulation_time) * ones(size(time_vector));
        plot(time_vector, power, 'g-', 'LineWidth', 2);
        title('Energy Consumption Rate', 'FontSize', 12);
        xlabel('Time (s)');
        ylabel('Power');
        grid on;
        
        % Inter-Robot Distance Distribution
        subplot(2,3,5);
        distances = pdist(robots_pos);
        histogram(distances, 20, 'FaceColor', [0.2, 0.4, 0.9], ...
                  'EdgeColor', 'black');
        title('Inter-Robot Distance Distribution', 'FontSize', 12);
        xlabel('Distance');
        ylabel('Frequency');
        grid on;
        
        % Robot Speed Distribution
        subplot(2,3,6);
        speeds = sqrt(sum(robots_vel.^2, 2));
        histogram(speeds, 15, 'FaceColor', [0.8, 0, 0.8], ...
                  'EdgeColor', 'black');
        title('Robot Speed Distribution', 'FontSize', 12);
        xlabel('Speed');
        ylabel('Frequency');
        grid on;
    end
end
