% Continue training old agent to reduce the horizontal position error
trainOpts = rlTrainingOptions;
trainOpts.StopTrainingCriteria = "AverageReward";
trainOpts.StopTrainingValue = 1800;
trainOpts.SaveAgentCriteria = 'AverageReward';
trainOpts.SaveAgentValue = 1800;
trainOpts.MaxEpisodes = 9000;        
trainOpts.MaxStepsPerEpisode = 400;
trainOpts.Plots = "training-progress";
trainOpts.UseParallel = true;
trainOpts.ScoreAveragingWindowLength = 100;
trainOpts.ParallelizationOptions.Mode = "async";

disp('Resuming training on existing agent (saved_agent)...')
trainingStats = train(saved_agent, env, trainOpts);