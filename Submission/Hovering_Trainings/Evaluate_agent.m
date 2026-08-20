% Batch evaluation of the trained hover agent.
 
n_trials = 30;
max_steps = 400;   % matches trainOpts.MaxStepsPerEpisode
dt = 0.05;
target = [0; 0; 5];
 
agent.UseExplorationPolicy = false; % evaluate the deterministic policy
 
final_pos_error = zeros(n_trials,1);
final_height_error = zeros(n_trials,1);
final_horiz_error  = zeros(n_trials,1);
final_x   = zeros(n_trials,1);
final_y   = zeros(n_trials,1);
final_psi = zeros(n_trials,1);
crashed         = false(n_trials,1);
steps_survived  = zeros(n_trials,1);
start_z         = zeros(n_trials,1);
 
for trial = 1:n_trials
    % Sample a start state exactly like Reset_Function.m does
    current_state = zeros(12,1);
    current_state(3) = 1 + rand*7;      % z   in [1, 8]
    current_state(1) = (rand-0.5)*2;    % x   in [-1, 1]
    current_state(2) = (rand-0.5)*2;    % y   in [-1, 1]
    current_state(4) = (rand-0.5)*0.5;  % u
    current_state(5) = (rand-0.5)*0.5;  % v
    current_state(6) = (rand-0.5)*0.5;  % w
    current_state(7) = (rand-0.5)*0.1;  % phi
    current_state(8) = (rand-0.5)*0.1;  % theta
 
    start_z(trial) = current_state(3);
    did_crash = false;
    step = 0;
 
    for step = 1:max_steps
        action_cell = getAction(saved_agent, current_state);
        action = action_cell{1};
        next_state = physics_quadrocopter(current_state, action, dt);
 
        z = next_state(3); x = next_state(1); y = next_state(2);
        phi = next_state(7); theta = next_state(8);
        pqr = next_state(10:12);
 
        % Same termination logic as training/Reward_function.
        if z <= 0.01 || z > 15 || abs(phi) > 1.05 || abs(theta) > 1.05 ...
                || abs(x) > 10 || abs(y) > 10 || max(abs(pqr)) > 15
            did_crash = true;
            break;
        end
        current_state = next_state;
    end
 
    crashed(trial)         = did_crash;
    steps_survived(trial)  = step;
    final_pos_error(trial)    = norm(current_state(1:3) - target);
    final_height_error(trial) = current_state(3) - target(3);        % signed: negative = undershoot
    final_horiz_error(trial)  = norm(current_state(1:2) - target(1:2));
    final_x(trial)   = current_state(1);
    final_y(trial)   = current_state(2);
    final_psi(trial) = current_state(9);
 
    fprintf('Trial %2d: start z=%5.2f m | final xyz=(%5.2f, %5.2f, %5.2f) | psi=%+5.2f rad | height err=%+5.2f m | horiz err=%5.2f m | crashed=%d | steps=%3d/%3d\n', ...
        trial, start_z(trial), current_state(1), current_state(2), current_state(3), current_state(9), ...
        final_height_error(trial), final_horiz_error(trial), did_crash, step, max_steps);
end
 
fprintf('\n--- Summary over %d trials ---\n', n_trials);
fprintf('Crash rate:                    %.1f%%\n', 100*mean(crashed));
fprintf('Mean final position error:     %.2f m\n', mean(final_pos_error));
fprintf('Median final position error:   %.2f m\n', median(final_pos_error));
fprintf('Worst-case position error:     %.2f m\n', max(final_pos_error));
fprintf('Std dev of position error:     %.2f m\n', std(final_pos_error));
fprintf('\n');
fprintf('Mean SIGNED height error:      %+.2f m  (negative = consistently undershooting target_z)\n', mean(final_height_error));
fprintf('Std dev of height error:       %.2f m\n', std(final_height_error));
fprintf('Mean horizontal error (x,y):   %.2f m\n', mean(final_horiz_error));
fprintf('Std dev of horizontal error:   %.2f m\n', std(final_horiz_error));
 
