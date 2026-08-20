% Use of an SAC algorithm to train the environment. Effective state of the art 
% agent used for continuous action systems. As the drone application has
% continuous thrust values. It shows a fast convergence and a better
% performance. The entropy maximization included in this algorithm enhances
% the stability and exploration.
% Sources: 

% A Cutting-Edge Energy Management System for a Hybrid Electric Vehicle relying on Soft Actor–Critic Deep Reinforcement Learning
% https://www.sciencedirect.com/science/article/pii/S2666691X25000089

% Matlab: Soft Actor-Critic (SAC) Agent
% https://de.mathworks.com/help/reinforcement-learning/ug/soft-actor-critic-agents.html

%%
% Define the agent settings
agentOpts = rlSACAgentOptions;
agentOpts.SampleTime = 0.05;              % Same as in reward function
agentOpts.DiscountFactor = 0.99;          % Prioritize long-term success
agentOpts.ExperienceBufferLength = 1e6;   % Safe a million state transitions
agentOpts.MiniBatchSize = 256;            % Used experience for update of network 

max_steps = 400;                          % Define how many steps per episode


% create agent
% Matlab documentation:
% https://de.mathworks.com/help/reinforcement-learning/ref/rl.agent.rlsacagent.html

agent = rlSACAgent(obsInfo, actInfo, agentOpts);         

trainOpts = rlTrainingOptions;
trainOpts.MaxEpisodes = 9000;
trainOpts.MaxStepsPerEpisode = max_steps;
trainOpts.StopTrainingCriteria = "AverageReward";
trainOpts.StopTrainingValue = 1400;
trainOpts.Plots = "training-progress";
trainOpts.SaveAgentCriteria = 'AverageReward';
trainOpts.SaveAgentValue = 1400;
trainOpts.UseParallel  =true;
trainOpts.ScoreAveragingWindowLength = 30;
trainOpts.ParallelizationOptions.Mode = "async";

% Start training
disp('Start training')
trainingStats = train(agent, env, trainOpts);