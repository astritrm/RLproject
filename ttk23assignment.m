clear; clc;

%Q-learning parameters:
e_max = 1000; %Maximum cross-track error considered
chi_max = 2*pi; %Maximum course value
de = 20; %Cross-track error step-threshold
dchi = deg2rad(10); %Course step-threshold
dt = 10; %Time step
% M = 1000; %Number of steps, replaced by tsamp in pathfollowing.m
N = 14; %Number of episodes/iterations of Q-learning

%chi = psi - alpha_k
N_a = 3; % Number of actions, meaning controllers(delta-value)
Q = zeros(ceil(e_max/de)+1, ceil(chi_max/dchi)+1, N_a); %Initializes Q with 

run pathfollowing
options = simset('SrcWorkspace','current');

%Begins the Q-learning loop - it continues in the guidance-block in
%MSfartoystyring26
for i = 1:N %Number of episodes 
%     state.p = [1500 500]; %Initial position
%     state.v = [6.63 0]'; %Initial velocity
%     state.psi = deg2rad(50); %Initial yaw angle
%     state.r = 0; %Initial yaw rate
    sim('MSFartoystyring26',[],options)
%     [e, chi, state] = pathfollowing(state,600,dt);
    %{
    for j = 1:M %Number of steps per episode
        %[e_tp, chi_tp] =sim(e_tp,chi_tp,a);
        %[e_tp, chi_tp] = run task 2_6 1 timestep with action a
        
        e_current = min(round(abs(e)/de), e_max)+1;
        chi_current = round(chi/dchi)+1;
        
        [value, a] = max(Q(e_current,chi_current,:));
        indices = find(Q(e_current,chi_current,:) == value);
        if (length(indices) > 1) || (rand() > epsilon)
            a = randi([1,3]);
        end
        
        action = deltas(a);
        [e_next, chi_next, state] = pathfollowing(state,action,dt);
        
        Q(e_current, chi_current, a) = Q(e_current, chi_current, a)...
            + alpha*(R(e) + gamma*max(Q(e_next,chi_next,:)) - Q(e_current, chi_current, a));
        e = e_next;
        chi = chi_next;
    end
    %}
    Q = Q_out.data(:,:,:,end);
end

%% Path Plot
%close all;

dec = 20;
object_tracking = 0;
pathplotter(p(:,1), p(:,2), psi, tsamp, dec, tstart, tstop, object_tracking, WP);

%%Use the manual switch in simulink to compare between sideslip-correction and no
%%correction
chi = beta(1:length(psi))+psi;
betap = beta(1:length(psi));
figure()
plot(t,rad2deg*psi,t,rad2deg*chi,t, rad2deg*chi_d,'LineWidth', 2);
legend('\psi','\chi','\chi_d', 'Interpreter', 'latex');

figure()
plot(t, rad2deg*betap, 'LineWidth', 2);
legend('\beta');

figure()
plot(t, v(:,1), 'LineWidth', 2);
legend('u');

