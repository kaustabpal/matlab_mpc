function [controls] = predict_controls(plan_horizon,agent_state,goal_state,vp,wp,v0,w0,dt)
    w1 = 1; % predict goal - actual goal
    w2 = 10; % difference between velocities
    w3 = 20; % difference between angular velocities
    w4 = 500; % first control velocity must be same as last velocity
    cost = @(u) w1*(norm(predict_goal(u,agent_state,dt,plan_horizon)-goal_state)) ...
        + w2*(norm(diff(u(:,1)))) + w3*(norm(diff(u(:,2)))) + w4*(norm(u(1,1)-vp)+norm(u(1,2)-wp)); 
    

    amin = -3; % min acceleration
    amax = 2; % max acceleration
    alphamax = 0.1; % max angular acceleration
    alphamin = -0.1; % min angular acceleration
    
    A = [diff(eye(plan_horizon)), zeros(plan_horizon-1,plan_horizon)];
    A = [A; -A];
    A = [A; [zeros(plan_horizon-1,plan_horizon) diff(eye(plan_horizon))]];
    A = [A; [zeros(plan_horizon-1,plan_horizon) -diff(eye(plan_horizon))]];
    b = [amax*dt*ones(plan_horizon-1,1);-amin*dt*ones(plan_horizon-1,1); alphamax*dt*ones(plan_horizon-1,1);-alphamin*dt*ones(plan_horizon-1,1)];
    
    v_ulim = 20*ones(plan_horizon,1); % max velocity = 20
    w_ulim = 0.5*ones(plan_horizon,1); % max abgular vel = 0.5
    v_llim = 0*ones(plan_horizon,1); % min vel = 0
    w_llim = -0.5*ones(plan_horizon,1); % min angular vel = -0.5
    ub = [v_ulim w_ulim];
    lb = [v_llim w_llim];
    options = optimoptions(@fmincon,'Display','iter');
    controls = fmincon(cost,[v0,w0],A,b,[],[],lb,ub,[],options);
end