% same training environment (only reward and reset function changed)

obsInfo = rlNumericSpec([12 1]);
obsInfo.Name = 'Quadcopter States (relative to platform, landing task)';

actInfo = rlNumericSpec([4 1], 'LowerLimit', 0, 'UpperLimit', 5);
actInfo.Name = 'Rotor Thrusts';

env = rlFunctionEnv(obsInfo, actInfo, "Reward_function_Landing", "Reset_Function_Landing");

validateEnvironment(env);
disp('Landing environment validated');