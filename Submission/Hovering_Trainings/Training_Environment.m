% Define observation space (states)
obsInfo = rlNumericSpec([12 1]);
obsInfo.Name = 'Quadcopter States';

% Define the Action space (propulsions)
% Defined an upplerlimit of the propulsions (thrust of 750 grams per rotor)
% Matlab documentation:
% https://de.mathworks.com/help/reinforcement-learning/ref/rl.util.rlnumericspec.html
actInfo = rlNumericSpec([4 1], 'LowerLimit', 0, 'UpperLimit', 5);
actInfo.Name = 'Rotor Thrusts';

% Build up environment with reward and reset function
% Matlab documentation:
%https://de.mathworks.com/help/reinforcement-learning/ref/rl.env.rlfunctionenv.html
env = rlFunctionEnv(obsInfo, actInfo, "Reward_function_Quadrocopter", "Reset_Function");

% Validate learning environemnt
% Matlab documentation
% (https://de.mathworks.com/help/reinforcement-learning/ref/rl.env.basicgridworld.validateenvironment.html)
validateEnvironment(env);
disp('Environemnt validated');