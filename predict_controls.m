function [controls] = predict_controls(plan_horizon,agent_state,goal_state,v0,w0,dt)
    
    cost = @(u)norm(predict_goal(u,agent_state,dt,plan_horizon)-goal_state);
    v_ulim = 20*ones(plan_horizon,1); % max velocity = 10
    w_ulim = 0.1*ones(plan_horizon,1); % max abgular vel = 0.1
    v_llim = 0*ones(plan_horizon,1); % min vel = 0
    w_llim = -0.1*ones(plan_horizon,1); % min angular vel = -0.1
    ub = [v_ulim w_ulim];
    lb = [v_llim w_llim];
    options = optimoptions(@fmincon,'Display','iter');
    controls = fmincon(cost,[v0,w0],[],[],[],[],lb,ub,[],options);
end