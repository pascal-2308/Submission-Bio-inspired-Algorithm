function [Observation, Reward, IsDone, LoggedSignals] = Reward_function_Quadrocopter(Action, LoggedSignals)

% Define current/ next state and timesteps for the physics model
current_state = LoggedSignals.State;
dt = 0.05;
next_state = physics_quadrocopter(current_state, Action, dt);


x = next_state(1);
y = next_state(2);
u = next_state(4); 
v = next_state(5); 
z = next_state(3);
w = next_state(6);
phi = next_state(7);
theta = next_state(8);
p     = next_state(10);
q     = next_state(11);
r     = next_state(12);

% Design reward function (initial values)
target_z = 5.0;
Reward = 3; 
z_error = target_z -z;
Reward = Reward - 0.2 * (z_error^2);

% Penalty for high rotation angles
Reward = Reward - (phi^2 + theta^2) * 0.1;

% Penalty for high velocities
Reward = Reward - 0.15 * (w^2);

% Penalty for high angular rates
Reward = Reward - 0.01 * (p^2 + q^2 + r^2);

% Penalty for horizontal velocities and position errors
Reward = Reward - 0.4 * (x^2 + y^2);
Reward = Reward - 0.5 * (u^2 + v^2);

% Define position error
pos_error_sq = x^2 + y^2 + z_error^2;
vel_error_sq = u^2 + v^2 + w^2;
att_error_sq = phi^2 + theta^2;

hover_bonus = 3 * exp(-pos_error_sq / (2*0.8^2)) ...
    * exp(-vel_error_sq / (2*0.6^2)) ...
    * exp(-att_error_sq / (2*0.2^2));

Reward = Reward + hover_bonus;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Reward = max(Reward, -30);

IsDone = false;

Crash_penalty = -50;

% Penalty if drone touches ground
if z <= 0.01 
    IsDone = true;
    Reward = Crash_penalty; 
end

% Penalty if out of control
if z > 15 || abs(phi) > 1.05 || abs(theta) > 1.05
    IsDone = true;
    Reward = Crash_penalty;
end

% Penalty if drifting too far away
if abs(x) > 10 || abs(y) > 10
    IsDone = true;
    Reward = Crash_penalty;
end

% Penalty for uncontrollable spin
if max(abs([p, q, r])) > 15
    IsDone = true;
    Reward = Crash_penalty;
end


% Next state definition
Observation = next_state;
LoggedSignals.State = next_state;

end