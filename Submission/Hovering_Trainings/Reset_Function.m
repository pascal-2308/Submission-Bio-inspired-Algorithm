function [InitialObservation, LoggedSignals] = Reset_Function()

% reset to initial observation
InitialObservation = zeros(12,1);
InitialObservation(3) = 1 + rand * 7; % z

% add certain noise for horizontal variation
InitialObservation(1) = (rand - 0.5) * 2; % x
InitialObservation(2) = (rand - 0.5) * 2; % y

% add certain noise in velocities
InitialObservation(4) = (rand - 0.5) * 0.5; % u
InitialObservation(5) = (rand - 0.5) * 0.5; % v
InitialObservation(6) = (rand - 0.5) * 0.5; % w

% add certain noises for a robust learning model
InitialObservation(7) = (rand - 0.5) * 0.1; % phi
InitialObservation(8) = (rand - 0.5) * 0.1; % theta

LoggedSignals.State = InitialObservation;

end