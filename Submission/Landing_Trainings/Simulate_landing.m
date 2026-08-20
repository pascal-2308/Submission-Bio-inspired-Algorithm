% Visualises a single landing attempt and produces the trajectory figure
% used in the report.



% some settings and to make sure it always matches the reset function as I
% changed some values manually over the different trainings
seed = 7;            % fixed, so the figure in the report is reproducible
save_fig = true;

opts.dt             = 0.05;
opts.max_steps      = 400;
opts.gear_height    = 0.05;
opts.height_offset  = 0.0;    % match Reset_Function_Landing.m
opts.platform_speed = 0.5;    % match Reset_Function_Landing.m

% same tolerances as Reward_function_Landing.m and Evaluate_landing.m
opts.land_horiz_tol = 0.2;    % m
opts.land_vel_tol   = 0.25;   % m/s
opts.land_tilt_tol  = 0.15;   % rad

% starting point
rng(seed);

% use loaded agent
agent = saved_agent;
agent.UseExplorationPolicy = false;   % no random exploration

main = fly_once(agent, opts);

fprintf('Outcome: %s  (%d steps, t = %.2f s)\n', main.outcome, main.n, main.n*opts.dt);
if main.touched
    fprintf('  sink rate at contact : %.3f m/s   (tolerance %.2f)\n', main.sink, opts.land_vel_tol);
    fprintf('  relative speed       : %.3f m/s   (tolerance %.2f)\n', main.rel_speed, opts.land_vel_tol);
    fprintf('  horizontal miss      : %.3f m     (tolerance %.2f)\n', main.miss, opts.land_horiz_tol);
    fprintf('  tilt                 : %.3f rad   (tolerance %.2f)\n', main.tilt, opts.land_tilt_tol);
else
    fprintf('  no touchdown in this attempt\n');
end


% create figure to plot 
figure('Name','Landing attempt','Position',[80 60 900 700]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% define colors
col_drone = [0.10 0.30 0.70];
col_plat  = [0.10 0.60 0.25];
col_faint = [0.75 0.78 0.85];

% 3D trajectory
nexttile; hold on; grid on; box on;

plot3(main.plat(:,1), main.plat(:,2), main.plat(:,3), '-', ...
    'Color', col_plat, 'LineWidth', 2, 'DisplayName','platform path');
plot3(main.drone(:,1), main.drone(:,2), main.drone(:,3), '-', ...
    'Color', col_drone, 'LineWidth', 1.8, 'DisplayName','drone path');
plot3(main.drone(1,1), main.drone(1,2), main.drone(1,3), 'o', ...
    'MarkerSize', 8, 'MarkerFaceColor','w', 'Color', col_drone, 'DisplayName','start');
plot3(main.drone(end,1), main.drone(end,2), main.drone(end,3), 'p', ...
    'MarkerSize', 14, 'MarkerFaceColor', col_drone, 'Color', col_drone, 'DisplayName','touchdown');

% landing tolerance circle on the deck, at the platform position at contact
th = linspace(0, 2*pi, 60);
cx = main.plat(end,1) + opts.land_horiz_tol*cos(th);
cy = main.plat(end,2) + opts.land_horiz_tol*sin(th);
plot3(cx, cy, zeros(size(th)), '--', 'Color', col_plat, 'LineWidth', 1.2, ...
    'DisplayName', sprintf('tolerance %.2f m', opts.land_horiz_tol));

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('(a) Trajectory - %s', main.outcome), 'FontWeight','normal');
legend('Location','northeast');
view(40, 22); axis tight; grid on;

% height over time
nexttile; hold on; grid on; box on;
plot(main.t, main.drone(:,3), '-', 'Color', col_drone, 'LineWidth', 1.8);
yline(opts.gear_height, '--', 'resting height on skids');
yline(0, ':', 'deck');
if main.touched
    plot(main.t(end), main.drone(end,3), 'p', 'MarkerSize', 12, ...
        'MarkerFaceColor', col_drone, 'Color', col_drone);
end
xlabel('Time [s]'); ylabel('Height Z [m]');
title('(b) Height above the deck', 'FontWeight','normal');

% horizontal distance to the platform
nexttile; hold on; grid on; box on;
plot(main.t, main.horiz, '-', 'Color', col_drone, 'LineWidth', 1.8);
yline(opts.land_horiz_tol, 'r--', sprintf('tolerance %.2f m', opts.land_horiz_tol));
xlabel('Time [s]'); ylabel('Horizontal distance [m]');
title('(c) Horizontal distance to the platform', 'FontWeight','normal');

% relative speed
nexttile; hold on; grid on; box on;
plot(main.t, main.relspd, '-', 'Color', col_drone, 'LineWidth', 1.8);
yline(opts.land_vel_tol, 'r--', sprintf('tolerance %.2f m/s', opts.land_vel_tol));
xlabel('Time [s]'); ylabel('Speed relative to platform [m/s]');
title('(d) Relative speed', 'FontWeight','normal');

if main.touched
    sgtitle(sprintf(['Landing attempt: %s  |  miss %.3f m, sink %.3f m/s, ' ...
        'tilt %.3f rad  |  touchdown after %.2f s'], main.outcome, main.miss, ...
        main.sink, main.tilt, main.n*opts.dt), 'FontSize', 11);
end

if save_fig
    exportgraphics(gcf, 'landing_trajectory.png', 'Resolution', 300);
    savefig(gcf, 'landing_trajectory.fig');
    fprintf('\nFigure written to landing_trajectory.png\n');
end

% function to generate flight path 
function out = fly_once(agent, opts)


drone_state = zeros(12,1);
drone_state(3) = 1 + rand*7;
drone_state(1) = (rand-0.5)*2;
drone_state(2) = (rand-0.5)*2;
drone_state(4) = (rand-0.5)*0.5;
drone_state(5) = (rand-0.5)*0.5;
drone_state(6) = (rand-0.5)*0.5;
drone_state(7) = (rand-0.5)*0.1;
drone_state(8) = (rand-0.5)*0.1;

platform_pos = [0;0;0];
heading = rand*2*pi;
speed   = rand*opts.platform_speed;
platform_vel = [speed*cos(heading); speed*sin(heading); 0];

N = opts.max_steps;
drone  = zeros(N,3);
plat   = zeros(N,3);
horiz  = zeros(N,1);
relspd = zeros(N,1);

outcome = "timeout";
touched = false;
miss = NaN; sink = NaN; tilt = NaN; rel_speed = NaN;
n = 0;

for step = 1:N
    obs = relative_observation(drone_state, platform_pos, platform_vel, opts.height_offset);
    action_cell = getAction(agent, obs);
    action = action_cell{1};

    [drone_state, contact] = physics_quadrocopter_contact( ...
        drone_state, action, opts.dt, opts.gear_height);
    platform_pos = platform_pos + platform_vel*opts.dt;

    n = step;
    drone(step,:) = drone_state(1:3)';
    plat(step,:)  = platform_pos';

    dx = drone_state(1)-platform_pos(1);
    dy = drone_state(2)-platform_pos(2);
    horiz(step)  = sqrt(dx^2 + dy^2);
    relspd(step) = norm(drone_state(4:6) - platform_vel);

    phi = drone_state(7); theta = drone_state(8);
    pqr = drone_state(10:12);

    if contact.touched
        % everything judged at the impact, from the pre-clamp state
        touched   = true;
        miss      = sqrt((contact.pos(1)-platform_pos(1))^2 + ...
                         (contact.pos(2)-platform_pos(2))^2);
        rel_speed = norm(contact.vel - platform_vel);
        sink      = contact.sink;
        tilt      = max(abs(contact.att(1)), abs(contact.att(2)));

        % same classification as Evaluate_landing.m
        if miss < opts.land_horiz_tol
            if rel_speed < opts.land_vel_tol && tilt < opts.land_tilt_tol
                outcome = "LANDED";
            else
                outcome = "HARD LANDING";
            end
        else
            outcome = "MISSED PLATFORM";
        end
        break;
    end

    % same crash limits as the reward function
    if drone_state(3) > 15 || abs(phi) > 1.05 || abs(theta) > 1.05 ...
            || abs(dx) > 10 || abs(dy) > 10 || max(abs(pqr)) > 15
        outcome = "CRASH";
        break;
    end
end

out.drone     = drone(1:n,:);
out.plat      = plat(1:n,:);
out.horiz     = horiz(1:n);
out.relspd    = relspd(1:n);
out.t         = (0:n-1)'*opts.dt;
out.n         = n;
out.outcome   = outcome;
out.touched   = touched;
out.miss      = miss;
out.sink      = sink;
out.tilt      = tilt;
out.rel_speed = rel_speed;
end
