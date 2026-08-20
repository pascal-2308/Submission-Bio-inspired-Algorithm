% Reset function for platform
function [InitialObservation, LoggedSignals] = Reset_Function_Platform()

% The same randomization as for the hovering 
drone_state = zeros(12,1);
drone_state(3) = 1 + rand * 7;       % z
drone_state(1) = (rand - 0.5) * 2;   % x
drone_state(2) = (rand - 0.5) * 2;   % y
drone_state(4) = (rand - 0.5) * 0.5; % u
drone_state(5) = (rand - 0.5) * 0.5; % v
drone_state(6) = (rand - 0.5) * 0.5; % w
drone_state(7) = (rand - 0.5) * 0.1; % phi
drone_state(8) = (rand - 0.5) * 0.1; % theta

% Define a platform speed
platform_speed = 1.1;

% Random initial conditions for platform behavior
platform_pos = [0;0; 0];
heading = rand * 2*pi;
speed   = rand * platform_speed;
platform_vel = [speed*cos(heading); speed*sin(heading); 0];
height_offset = 5;

LoggedSignals.State        = drone_state;
LoggedSignals.PlatformPos  = platform_pos;
LoggedSignals.PlatformVel  = platform_vel;
LoggedSignals.HeightOffset = height_offset;

InitialObservation = relative_observation(drone_state, platform_pos, platform_vel, height_offset);

end