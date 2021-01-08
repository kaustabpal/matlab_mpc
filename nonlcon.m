function [c, ceq] = nonlcon(controls,plan_horizon,agent_state,agent_radius,obstacle_radius,obstacle_pos,obs_v,obs_w,dt)
    radi_sum = (agent_radius + obstacle_radius)*ones(plan_horizon,1);
    agnt_obs_dist = zeros(plan_horizon,1);
    
    x = agent_state(1);
    y = agent_state(2);
    theta = agent_state(3);
    
    new_state = agent_state;
    
    for i=1:plan_horizon
      new_state = nonhn_update(new_state, controls(i,1), controls(i,2), dt);
      obs = nonhn_update(obstacle_pos, obs_v,obs_w,dt);
      agnt_obs_dist(i,1) = norm(obs-new_state);     
      
    end
    c = radi_sum - agnt_obs_dist;
    ceq = [];
end