function [Observation, Reward, IsDone, LoggedSignals] = Reward_function_Platform(Action, LoggedSignals)

% Get states from the reset function and define quadrocopter states
drone_state   = LoggedSignals.State;
platform_pos  = LoggedSignals.PlatformPos;
platform_vel  = LoggedSignals.PlatformVel;
height_offset = LoggedSignals.HeightOffset;
dt = 0.05;

next_drone_state  = physics_quadrocopter(drone_state, Action, dt);
next_platform_pos = platform_pos + platform_vel * dt;  

% I changed the position states now to relative positions to train the next
% agent to track the position rather than landing already

x = next_drone_state(1) - next_platform_pos(1);
y = next_drone_state(2) - next_platform_pos(2);
z_error = (next_platform_pos(3) + height_offset) - next_drone_state(3);
u = next_drone_state(4) - platform_vel(1);
v = next_drone_state(5) - platform_vel(2);
w = next_drone_state(6) - platform_vel(3);
phi = next_drone_state(7);
theta = next_drone_state(8);
p = next_drone_state(10); 
q = next_drone_state(11); 
r = next_drone_state(12);

% Unchanged to reward definition of the hovering training only that now the
% input values are rather relative to the platform
Reward = 3;
Reward = Reward - 0.2 * (z_error^2);
Reward = Reward - (phi^2 + theta^2) * 0.1;
Reward = Reward - 0.15 * (w^2);
Reward = Reward - 0.01 * (p^2 + q^2 + r^2);
Reward = Reward - 0.4 * (x^2 + y^2);
Reward = Reward - 0.5 * (u^2 + v^2);

pos_error_sq = x^2 + y^2 + z_error^2;
vel_error_sq = u^2 + v^2 + w^2;
att_error_sq = phi^2 + theta^2;
hover_bonus = 3 * exp(-pos_error_sq / (2*0.8^2)) ...
    * exp(-vel_error_sq / (2*0.6^2)) ...
    * exp(-att_error_sq / (2*0.2^2));
Reward = Reward + hover_bonus;

Reward = max(Reward, -30);

IsDone = false;
Crash_penalty = -50;

z_abs = next_drone_state(3);
if z_abs <= 0.01
    IsDone = true; Reward = Crash_penalty;
end
if z_abs > 15 || abs(phi) > 1.05 || abs(theta) > 1.05
    IsDone = true; Reward = Crash_penalty;
end
if abs(x) > 10 || abs(y) > 10  
    IsDone = true; Reward = Crash_penalty;
end
if max(abs([p, q, r])) > 15
    IsDone = true; Reward = Crash_penalty;
end

% Log new states for training
Observation = relative_observation(next_drone_state, next_platform_pos, platform_vel, height_offset);
LoggedSignals.State       = next_drone_state;
LoggedSignals.PlatformPos = next_platform_pos;
LoggedSignals.PlatformVel = platform_vel;  
LoggedSignals.HeightOffset = height_offset;

end