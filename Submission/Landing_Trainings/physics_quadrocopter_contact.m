function [next_state, contact] = physics_quadrocopter_contact(current_state, actions, dt, gear_height)

% later defined in training to allow agent to learn that there is a bit of
% a magring to the ground as a landing gear is available. Made it easier
% for the training 
if nargin < 4 || isempty(gear_height)
    gear_height = 0.05;      % m, skid length below the body centre
end

motor_thrusts = min(max(actions(:), 0), 5);

n_sub  = 10;
dt_sub = dt / n_sub;

contact = struct('touched', false, 'vel', zeros(3,1), 'sink', 0, ...
                 'pos', zeros(3,1), 'att', zeros(3,1));

state = current_state;
% evaluate for each substep the outcomes evaluated in step_dynamics function
for k = 1:n_sub
    [state, hit, pre] = step_dynamics(state, motor_thrusts, dt_sub, gear_height);
    if hit && ~contact.touched
        contact.touched = true;
        contact.vel = pre(4:6);
        contact.sink = max(0, -pre(6));
        contact.pos = pre(1:3);
        contact.att = pre(7:9);
    end
end
next_state = state;

end


function [next_state, hit, pre_clamp] = step_dynamics(current_state, actions, dt, gear_height)

% some drone values for mass, moments of inertia etc...
params.m    = 0.486;
params.L    = 0.25;
params.g    = 9.81;
params.kd   = 0.01;
params.Ixx  = 4.856*1e-3;
params.Iyy  = 4.856*1e-3;
params.Izz  = 8.801*1e-3;
params.drag = 0.15;

% getting states from current evaluation
T1 = actions(1); T2 = actions(2); T3 = actions(3); T4 = actions(4);
u = current_state(4); v = current_state(5); w = current_state(6);
phi = current_state(7); theta = current_state(8); psi = current_state(9);
p = current_state(10); q = current_state(11); r = current_state(12);

T_total   = T1 + T2 + T3 + T4;
tau_phi   = (T4 - T2) * params.L;
tau_theta = (T1 - T3) * params.L;
tau_psi   = (T1 - T2 + T3 - T4) * params.kd;

% building up acceleration terms for horizontal and rotational dynamics
x_ddot = (T_total / params.m) * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) - (params.drag/params.m)*u;
y_ddot = (T_total / params.m) * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) - (params.drag/params.m)*v;
z_ddot = (T_total / params.m) * (cos(phi)*cos(theta)) - params.g - (params.drag/params.m)*w;

p_dot = (tau_phi   + (params.Iyy - params.Izz)*q*r) / params.Ixx;
q_dot = (tau_theta + (params.Izz - params.Ixx)*p*r) / params.Iyy;
r_dot = (tau_psi   + (params.Ixx - params.Iyy)*p*q) / params.Izz;

phi_dot   = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
theta_dot = cos(phi)*q - sin(phi)*r;
psi_dot   = sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r;

% evaluating next state with specified step width
next_state = zeros(12,1);

next_state(1) = current_state(1) + u * dt;
next_state(2) = current_state(2) + v * dt;
next_state(3) = current_state(3) + w * dt;

next_state(4) = u + x_ddot * dt;
next_state(5) = v + y_ddot * dt;
next_state(6) = w + z_ddot * dt;

next_state(7) = phi   + phi_dot   * dt;
next_state(8) = theta + theta_dot * dt;
next_state(9) = psi   + psi_dot   * dt;

next_state(10) = p + p_dot * dt;
next_state(11) = q + q_dot * dt;
next_state(12) = r + r_dot * dt;

% get state for later evaluation of hard/soft landing
pre_clamp = next_state;
hit = false;
if next_state(3) < gear_height
    hit = true;
    pre_clamp(3)   = gear_height;
    next_state(3)  = gear_height;
    next_state(6)  = max(0, next_state(6));
end

end
