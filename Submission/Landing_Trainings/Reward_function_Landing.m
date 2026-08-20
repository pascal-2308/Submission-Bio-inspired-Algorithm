function [Observation, Reward, IsDone, LoggedSignals] = Reward_function_Landing(Action, LoggedSignals)
 
% Define errors and states and the given rewards (only slightly changed
% from tracking reward function)
drone_state   = LoggedSignals.State;
platform_pos  = LoggedSignals.PlatformPos;
platform_vel  = LoggedSignals.PlatformVel;
height_offset = LoggedSignals.HeightOffset;
 
if isfield(LoggedSignals, 'GearHeight')
    gear_height = LoggedSignals.GearHeight;
else
    gear_height = 0.05;
end
 
dt = 0.05;
% get actaual state, define all errors with the drone state and platform state 
[next_drone_state, contact] = physics_quadrocopter_contact(drone_state, Action, dt, gear_height);
next_platform_pos = platform_pos + platform_vel * dt;
 
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

% start with reward calculation, penalties for wrong velo and so on
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
 
% I defined some measures to define what is a safe landing
touched_down = contact.touched;
land_horiz_tol  = 0.2;   % m
land_vel_tol    = 0.25;  % m/s
land_tilt_tol   = 0.15;  % rad

% see if the touchdown is a success or if the drone was to fast, tilted 
if touched_down
    IsDone = true;
 
    horiz_error = sqrt((contact.pos(1) - next_platform_pos(1))^2 + ...
        (contact.pos(2) - next_platform_pos(2))^2);
    rel_speed   = norm(contact.vel - platform_vel);
    phi         = contact.att(1);
    theta       = contact.att(2);
 
    use_graded = true;
 
    if use_graded
        d2 = (horiz_error / land_horiz_tol)^2 ...
            + (rel_speed   / land_vel_tol)^2 ...
            + (max(abs(phi),abs(theta)) / land_tilt_tol)^2;

        quality = exp(-d2 / 6);
 
        Reward  = Crash_penalty + (3000 - Crash_penalty) * quality;
    elseif horiz_error < land_horiz_tol && rel_speed < land_vel_tol ...
            && abs(phi) < land_tilt_tol && abs(theta) < land_tilt_tol
        Reward = 3000;
    else
        Reward = Crash_penalty; % touched ground but missed platform or it crashed with a too high velocity
    end
    Observation = relative_observation(next_drone_state, next_platform_pos, platform_vel, height_offset);
    LoggedSignals.State        = next_drone_state;
    LoggedSignals.PlatformPos  = next_platform_pos;
    LoggedSignals.PlatformVel  = platform_vel;
    LoggedSignals.HeightOffset = height_offset;
    LoggedSignals.GearHeight   = gear_height;
    return
end
 
% same crash checks as in tracking platform (still valid as drone should not
% experience unwanted behavior in the air)
z_abs = next_drone_state(3);
 
if z_abs > 15 || abs(phi) > 1.05 || abs(theta) > 1.05
    IsDone = true; Reward = Crash_penalty;
end
if abs(x) > 10 || abs(y) > 10
    IsDone = true; Reward = Crash_penalty;
end
if max(abs([p, q, r])) > 15
    IsDone = true; Reward = Crash_penalty;
end
 
Observation = relative_observation(next_drone_state, next_platform_pos, platform_vel, height_offset);
LoggedSignals.State        = next_drone_state;
LoggedSignals.PlatformPos  = next_platform_pos;
LoggedSignals.PlatformVel  = platform_vel;
LoggedSignals.HeightOffset = height_offset;
LoggedSignals.GearHeight   = gear_height;
 
end
