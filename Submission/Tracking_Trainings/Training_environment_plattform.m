% Define training environment for the platform 
% Same environment as hovering training

obsInfo = rlNumericSpec([12 1]);
obsInfo.Name = 'Quadcopter States (relative to platform)';

actInfo = rlNumericSpec([4 1], 'LowerLimit', 0, 'UpperLimit', 5);
actInfo.Name = 'Rotor Thrusts';

% changed to the reward functions defined for hovering
env = rlFunctionEnv(obsInfo, actInfo, "Reward_function_Platform", "Reset_Function_Platform");

validateEnvironment(env);
disp('Platform-tracking environment validated');