% 4 different outcomes possible
% not just crash/no-crash - "did it land" is now the real question:
%   - landed:       touched down gently, within tolerance
%   - hard_landing: reached the deck but too fast/tilted (a crash)
%   - crash:        out of bounds, excessive tilt/spin, hit the ground
%                    away from the platform, etc.
%   - timeout:      never reached the deck within max_steps (still
%                    tracking/hovering, just hasn't landed)
 
n_trials = 30;
max_steps = 400;
dt = 0.05;
 
% has to match the reset function
platform_speed = 0.5;   % match Reset_Function_Landing.m
height_offset  = 0.0;   % match Reset_Function_Landing.m

% I defined some tolerances for the landing
land_height_tol = 1e-9;
land_horiz_tol  = 0.2;
land_vel_tol    = 0.25;
land_tilt_tol   = 0.15;

% the agent should not explore further while evaluation
saved_agent.UseExplorationPolicy = false;
 
outcome         = strings(n_trials,1);
final_pos_error = zeros(n_trials,1);
steps_taken     = zeros(n_trials,1);
 
for trial = 1:n_trials
    % randomize the positions speeds and so on
    drone_state = zeros(12,1);
    drone_state(3) = 1 + rand*7;
    drone_state(1) = (rand-0.5)*2;
    drone_state(2) = (rand-0.5)*2;
    drone_state(4) = (rand-0.5)*0.5;
    drone_state(5) = (rand-0.5)*0.5;
    drone_state(6) = (rand-0.5)*0.5;
    drone_state(7) = (rand-0.5)*0.1;
    drone_state(8) = (rand-0.5)*0.1;
    
    % platform randomization
    platform_pos = [0; 0; 0];
    heading = rand*2*pi;
    speed   = rand*platform_speed;
    platform_vel = [speed*cos(heading); speed*sin(heading); 0];
 
    trial_outcome = "timeout";
    
    for step = 1:max_steps
        % get all the states of drone and platform
        obs = relative_observation(drone_state, platform_pos, platform_vel, height_offset);
        action_cell = getAction(saved_agent, obs);
        action = action_cell{1};
        [drone_state,contact]  = physics_quadrocopter_contact(drone_state, action, dt);
        platform_pos = platform_pos + platform_vel*dt;
    
        % relative posiiton error to platform
        x = drone_state(1) - platform_pos(1);
        y = drone_state(2) - platform_pos(2);
        u = drone_state(4) - platform_vel(1);
        v = drone_state(5) - platform_vel(2);
        w = drone_state(6) - platform_vel(3);
        phi = drone_state(7); theta = drone_state(8);
        pqr = drone_state(10:12);
 
        % Touchdown check - against the ACTUAL deck      
        deck_height_error = drone_state(3) - platform_pos(3);
        horiz_error = sqrt(x^2 + y^2);
        rel_speed   = sqrt(u^2 + v^2 + w^2);
        
        % check if tolerances are achieved
        if contact.touched
            horiz_error = sqrt((contact.pos(1)-platform_pos(1))^2 + ...
                (contact.pos(2)-platform_pos(2))^2);
            rel_speed   = norm(contact.vel - platform_vel);
            phi_c       = contact.att(1);
            theta_c     = contact.att(2);
            
            % check for outcome
            if horiz_error < land_horiz_tol
                if rel_speed < land_vel_tol && abs(phi_c) < land_tilt_tol ...
                        && abs(theta_c) < land_tilt_tol
                    trial_outcome = "landed";
                else
                    trial_outcome = "hard_landing";
                end
            else
                trial_outcome = "missed_platform";
            end
            fprintf('        [touchdown: sink %.3f m/s, miss %.3f m]\n', ...
                contact.sink, horiz_error);
            break;
        end

        z_abs = drone_state(3);
        if z_abs > 15 || abs(phi) > 1.05 || abs(theta) > 1.05 ...
                || abs(x) > 10 || abs(y) > 10 || max(abs(pqr)) > 15
            trial_outcome = "crash";
            break;
        end
    end

    outcome(trial)     = trial_outcome;
    steps_taken(trial) = step;
    final_target_z = platform_pos(3) + height_offset;
    final_pos_error(trial) = norm([drone_state(1)-platform_pos(1); ...
        drone_state(2)-platform_pos(2); ...
        drone_state(3)-final_target_z]);

    fprintf('Trial %2d: outcome=%-13s | steps=%3d | final pos error=%.2f m\n', ...
        trial, trial_outcome, step, final_pos_error(trial));
end
% summaries what the stats are for a small overview for me
fprintf('\n--- Summary (height_offset=%.2f m, platform_speed=%.2f m/s) ---\n', height_offset, platform_speed);
fprintf('Successful soft landings: %.1f%%\n', 100*mean(outcome=="landed"));
fprintf('Hard landings (too fast/tilted): %.1f%%\n', 100*mean(outcome=="hard_landing"));
fprintf('Other crashes: %.1f%%\n', 100*mean(outcome=="crash"));
fprintf('Timeouts (never reached the deck): %.1f%%\n', 100*mean(outcome=="timeout"));