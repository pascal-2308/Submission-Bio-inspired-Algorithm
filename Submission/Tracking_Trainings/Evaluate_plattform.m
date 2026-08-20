% Batch evaluation of the platform-tracking agent - same discipline as
% evaluate_agent.m from the hover stage

n_trials = 30;
max_steps = 400;
dt = 0.05;

% Set this to whatever curriculum stage I just trained/want to test
platform_speed_max = 1.1;

saved_agent.UseExplorationPolicy = false;

final_rel_pos_error    = zeros(n_trials,1);
final_rel_height_error = zeros(n_trials,1);
final_rel_horiz_error  = zeros(n_trials,1);
crashed                = false(n_trials,1);
trial_platform_speed   = zeros(n_trials,1);

for trial = 1:n_trials
    drone_state = zeros(12,1);
    drone_state(3) = 1 + rand*7;
    drone_state(1) = (rand-0.5)*2;
    drone_state(2) = (rand-0.5)*2;
    drone_state(4) = (rand-0.5)*0.5;
    drone_state(5) = (rand-0.5)*0.5;
    drone_state(6) = (rand-0.5)*0.5;
    drone_state(7) = (rand-0.5)*0.1;
    drone_state(8) = (rand-0.5)*0.1;

    platform_pos = [0; 0; 0];
    heading = rand*2*pi;
    speed   = rand*platform_speed_max;
    platform_vel = [speed*cos(heading); speed*sin(heading); 0];
    height_offset = 5.0;
    trial_platform_speed(trial) = speed;

    did_crash = false;
    for step = 1:max_steps
        obs = relative_observation(drone_state, platform_pos, platform_vel, height_offset);
        action_cell = getAction(saved_agent, obs);
        action = action_cell{1};

        drone_state  = physics_quadrocopter(drone_state, action, dt);
        platform_pos = platform_pos + platform_vel*dt;

        z = drone_state(3); phi = drone_state(7); theta = drone_state(8);
        pqr = drone_state(10:12);
        rel_x = drone_state(1) - platform_pos(1);
        rel_y = drone_state(2) - platform_pos(2);

        if z <= 0.01 || z > 15 || abs(phi) > 1.05 || abs(theta) > 1.05 ...
                || abs(rel_x) > 10 || abs(rel_y) > 10 || max(abs(pqr)) > 15
            did_crash = true;
            break;
        end
    end

    final_target_z = platform_pos(3) + height_offset;
    final_rel_pos_error(trial) = norm([drone_state(1)-platform_pos(1); ...
        drone_state(2)-platform_pos(2); ...
        drone_state(3)-final_target_z]);
    final_rel_height_error(trial) = drone_state(3) - final_target_z;
    final_rel_horiz_error(trial)  = norm([drone_state(1)-platform_pos(1); ...
        drone_state(2)-platform_pos(2)]);
    crashed(trial) = did_crash;

    fprintf('Trial %2d: platform speed=%.2f m/s | rel pos error=%.2f m | crashed=%d\n', ...
        trial, speed, final_rel_pos_error(trial), did_crash);
end

fprintf('\n--- Summary (platform_speed_max = %.2f m/s) ---\n', platform_speed_max);
fprintf('Crash rate:                       %.1f%%\n', 100*mean(crashed));
fprintf('Mean relative position error:     %.2f m\n', mean(final_rel_pos_error));
fprintf('Mean SIGNED relative height error:%+.2f m\n', mean(final_rel_height_error));
fprintf('Mean relative horizontal error:   %.2f m\n', mean(final_rel_horiz_error));

figure('Name','Platform tracking evaluation');
scatter(trial_platform_speed, final_rel_pos_error, 40, crashed, 'filled');
xlabel('This trial''s platform speed [m/s]');
ylabel('Final relative position error [m]');
title(sprintf('Platform tracking accuracy (platform\\_speed\\_max = %.2f)', platform_speed_max));
colorbar;
grid on;