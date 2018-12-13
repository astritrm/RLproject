function [chi_d, e, Q] = fcn(p, WP, time, chi, Q, asd)
    
    %% FOR Q-LEARNING
    R = @(x) -x^2; %Simple reward function, x will be the cross-track error going forward

    %Q-learning parameters:
    e_max = 1000; %Maximum cross-track error considered
    chi_max = 2*pi; %Maximum course value
    de = 20; %Cross-track error step-threshold
    dchi = deg2rad(10); %Course step-threshold
    dt = 10; %Time step
    
    e_greed = 0.3; %chance of picking random action instead of highest value
    gamma = 0.9; %discount
    alpha = 0.95; %learning rate

    deltas = [100, 600, 1000]; %The three LOS-vectors(the controller parameters) this Q-learning algorithm will switch between
                                %The goal is to find a switching behavior
                                %which performs better than a single,
                                %well-tuned controller.

    persistent j Delta e_prev chi_prev;  %Used to remember the LOS-vector(Delta), cross-track error
                                         % and course throughout the
                                         % simulink-loop
    if (isempty(j))
        j = 0;
    end
    if (isempty(Delta))
        Delta = 0;
    end
    if (isempty(e_prev))
        e_prev = 1;
    end
    if (isempty(chi_prev))
        chi_prev = 1;
    end

    %% GUIDANCE-block code from my old Guidance and Control-assignment
    %number of waypoints
    N = length(WP);

    % waypoint iteration
    persistent k;
    
    %waypoint iteration
    if (isempty(k))
        k = 1;
    end
    
    %reference waypoint
    xk_ref = WP(1, k);
    yk_ref = WP(2, k);
    
    %next waypoint
    xk_target = WP(1, k+1);
    yk_target = WP(2, k+1);
    
    
    if k ~= 6 && k ~= 1
        %calculating optimal turningradius
        lastWP = [WP(1,k-1); WP(2,k-1)];
        currentWP = [WP(1,k); WP(2,k)]; %way points
        nextWP = [WP(1,k+1); WP(2,k+1)];
        u = (lastWP - currentWP)/(norm(lastWP-currentWP));
        v = (nextWP - currentWP)/(norm(nextWP-currentWP)); %a,b,c
        w = (u + v)/(norm(u) + norm(v));
        a = acos((norm(v)^2 + norm(u+v)^2 - norm(u)^2)/(2*norm(v)*norm(u+v)));
        turningRadius = 900 / sin(a)*1.3;
    else
        turningRadius = 900*1.3;
    end
    
   
    alpha_k = atan2(yk_target-yk_ref, xk_target-xk_ref);
    
    Rp = [cos(alpha_k) -sin(alpha_k);
          sin(alpha_k) cos(alpha_k);];
      
    epsilon = Rp'*(p - [xk_ref yk_ref]');
    
    %R = sqrt(((xk_target-xk_ref)^2+(yk_target-yk_ref)^2))-epsilon(1,:); %Radius of acceptance
    chi_p = alpha_k;
    s = epsilon(1);
    e = epsilon(2);   %Cross-track error 

    %% Q LEARNING
    chi_pathrel = mod(chi + pi - alpha_k, 2*pi); %Course relative to the path between waypoints
    
    %Q-matrix update algorithm
    if floor(time/dt) >= j
        j = j + 1;
        %[e_tp, chi_tp] =sim(e_tp,chi_tp,a);
        %[e_tp, chi_tp] = run task 2_6 1 timestep with action a

        e_current = min(round(abs(e)/de), e_max/de)+1;  %The cross-track error's value will be =< e_max,
        chi_current = round((chi_pathrel)/dchi)+1;      % and both course and e are discretized to have each of their 
                                                        % step-values match the size of their respective Q-matrix indices

        [value, a] = max(Q(e_current,chi_current,:));   %Decides the best suited action(Delta) from Q,
        indices = find(Q(e_current,chi_current,:) == value);   % but will choose randomly if 2 or more
        if (length(indices) > 1) || (rand() < e_greed)         % actions are equal, or simply because
            a = randi([1,3]);                                  % the random exploration rolls high enough
        end

        Delta = deltas(a);       %Chooses the appropriate LOS-vector value depending on action a

        Q(e_prev, chi_prev, a) = Q(e_prev, chi_prev, a)...  %Updates Q's values for the next iteration
            + alpha*(R(e) + gamma*max(Q(e_current,chi_current,:)) - Q(e_prev, chi_prev, a));
        
        e_prev = e_current;
        chi_prev = chi_current;
    end
    %%
    chi_r = atan(-e/Delta);   %The controller using the various Delta-values
    chi_d = chi_p+chi_r;
    chi_d = mod(chi_d, 2*pi);
    
    
    if (norm(WP(:, k+1)-WP(:, k)) - s <= turningRadius) && (N-1 > k)
       k = k + 1;
    end
    
    
    
    
    
 
    [asd, time, e] %Debugging, shows which iteration, duration and current
    %cross-track error during training
end
