% The previous training showed me that a stop value which is chosen to
% little results in an unfinished agent. Therefore, I will stop the
% training manually in this round when the learning curve shows stagnation.
% The rest of this training agent is the same as in the previous training
trainOpts = rlTrainingOptions;
trainOpts.MaxEpisodes = 9000;
trainOpts.MaxStepsPerEpisode = 400;
trainOpts.StopTrainingCriteria = "AverageReward";
trainOpts.StopTrainingValue = 1e6; 
trainOpts.SaveAgentCriteria = 'AverageReward';
trainOpts.SaveAgentValue = 2200;    
trainOpts.UseParallel = true;
trainOpts.ScoreAveragingWindowLength = 100;
trainOpts.ParallelizationOptions.Mode = "async";
trainOpts.Plots = "training-progress";

disp('Start training landing (warm-started from platform-tracking agent)...')
trainingStats = train(saved_agent, env, trainOpts);
