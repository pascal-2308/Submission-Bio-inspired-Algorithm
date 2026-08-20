% Training the agent similar to hover process. I use the same agent from
% the hovering stage and now expanding the training to a hover over the
% platform objective. First a tracking objective and then a landing
% objective.
% Same setting are used as they seemed reliable after multiple improvements
% as in the hovering training

trainOpts = rlTrainingOptions;
trainOpts.MaxEpisodes = 9000;
trainOpts.MaxStepsPerEpisode = 400;
trainOpts.StopTrainingCriteria = "AverageReward";
trainOpts.StopTrainingValue = 2400;
trainOpts.SaveAgentCriteria = 'AverageReward';
trainOpts.SaveAgentValue = 2400;
trainOpts.UseParallel = true;
trainOpts.ScoreAveragingWindowLength = 100;
trainOpts.ParallelizationOptions.Mode = "async";
trainOpts.Plots = "training-progress";

disp('Start training platfrom tracking')
trainingStats = train(saved_agent, env, trainOpts);