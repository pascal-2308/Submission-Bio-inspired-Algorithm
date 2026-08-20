% Visualize the trained platform-tracking agent following (or not) the
% moving platform 

dt = 0.05;
max_steps = 400;

% Match whatever platform_speed_max I most recently trained/tested with
platform_speed_max = 1.1;

drone_state = zeros(12,1);
drone_state(3) = 1;  % start height

platform_pos = [0; 0; 0];
heading = rand * 2*pi;
speed   = rand * platform_speed_max;
platform_vel = [speed*cos(heading); speed*sin(heading); 0];
height_offset = 5.0;

saved_agent.UseExplorationPolicy = false;

drone_traj     = zeros(max_steps, 3);
platform_traj  = zeros(max_steps, 3);
n_logged = 0;

for step = 1:max_steps
    obs = relative_observation(drone_state, platform_pos, platform_vel, height_offset);
    action_cell = getAction(saved_agent, obs);
    action = action_cell{1};

    drone_state  = physics_quadrocopter(drone_state, action, dt);
    platform_pos = platform_pos + platform_vel * dt;

    n_logged = step;
    drone_traj(step, :)    = drone_state(1:3)';
    platform_traj(step, :) = platform_pos';

    z = drone_state(3); phi = drone_state(7); theta = drone_state(8);
    if z <= 0.01 || z > 15 || abs(phi) > 1.05 || abs(theta) > 1.05
        fprintf('Flight aborted at step %d (crash or loss of control)\n', step);
        break;
    end
end

drone_traj    = drone_traj(1:n_logged, :);
platform_traj = platform_traj(1:n_logged, :);

figure('Name', 'Platform tracking flight');

subplot(1,2,1);
plot(platform_traj(:,1), platform_traj(:,2), 'g-', 'LineWidth', 2); hold on;
plot(drone_traj(:,1), drone_traj(:,2), 'b-', 'LineWidth', 1.5);
plot(platform_traj(end,1), platform_traj(end,2), 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(drone_traj(end,1), drone_traj(end,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('X position [m]'); ylabel('Y position [m]');
title('Top-down view');
legend('Platform path', 'Drone path', 'Location', 'best');
axis equal; grid on;

subplot(1,2,2);
t = (0:n_logged-1) * dt;
plot(t, drone_traj(:,3), 'b-', 'LineWidth', 1.5); hold on;
yline(height_offset, 'g--', 'target height above platform');
xlabel('Time [s]'); ylabel('Height Z [m]');
title('Height over time');
grid on;