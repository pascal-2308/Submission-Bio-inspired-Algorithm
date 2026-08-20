% Start state
current_state = zeros(12,1);
current_state(3) = 7; % starting height
dt = 0.05;
agent.UseExplorationPolicy = false;

% 3D-Plot prepare
figure('Name', 'Quadrocopter 3D-Flug', 'NumberTitle', 'off');
hold on; grid on;
view(3); 
axis([-5 5 -5 5 0 10]); %
xlabel('X position [m]');
ylabel('Y position [m]');
zlabel('Height Z [m]');
title('Simulated flight path with agent');

% Goal height
[X_grid, Y_grid] = meshgrid(-10:10, -10:10);
Z_grid = ones(size(X_grid)) * 5;
surf(X_grid, Y_grid, Z_grid, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'g');

drone_marker = plot3(current_state(1), current_state(2), current_state(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
trajectory_line = animatedline('Color', 'b', 'LineWidth', 1.5);

disp('Flug startet!');

% loop for certian step size
for step = 1:400
   
    action_cell = getAction(saved_agent, current_state); 
    action = action_cell{1}; 
    next_state = physics_quadrocopter(current_state, action, dt);
    x = next_state(1);
    y = next_state(2);
    z = next_state(3);

    set(drone_marker, 'XData', x, 'YData', y, 'ZData', z); 
    addpoints(trajectory_line, x, y, z); 
    drawnow; % update picutre

    % termination criteria
    if z <= 0.01 || z > 15 ...
            || abs(next_state(7)) > 1.05 || abs(next_state(8)) > 1.05 ...
            || abs(x) > 10 || abs(y) > 10 ...
            || max(abs(next_state(10:12))) > 15
        disp(['Flug abgebrochen in Schritt ', num2str(step), ' (Absturz oder Kontrollverlust)']);
        break;
    end

    current_state = next_state;
    pause(0.05); 
end

if step == 400
    disp('Flug erfolgreich beendet! Die Drohne ist stabil geflogen.');
end