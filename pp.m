%Mitt elendige forsøk på path-planning + Q-learning

epsilon = 0.7; %chance of picking random action
gamma = 0.9; %discount
lambda = 0.9; %decay of epsilon
alpha = 0.95; %learning rate
threshold = 0; %threshold
successrate = 1;
lookahead = 30; %Design parameter

waypointReward = 30;
goalReward = 50;
episodes = 1000; % One episode: from start to goal state

rewardFunc = -(crosstrack^2);

u = arctan(-crosstrack/lookahead); %Velocity-path relative angle

a = u; %Actions decide which controller to use

%Q(s, a) = Q(s, a) + alpha*(r + gamma*max_a_next(Q(s_next, a_next)) - Q(s, a))