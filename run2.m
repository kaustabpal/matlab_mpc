close all;
clear all;
clc;
dt = 0.1;
init_state = [0,0,0]; % initial state [x,y,theta]
goal_state = [80,80,0.5]; % goal state [x,y,theta]
cur_state = init_state; % current state. Initialize as initial state.
vp = 0;  % starting velocity
wp = 0; % starting angular velocity

plan_horizon = 50; % plan for 50 timesteps
update_horizon = 10; % update controls after 20 timesteps. (20% of plan_horizon)

v0 = normrnd(5,2.5,plan_horizon,1); % initial guess of velocity
w0 = rand(plan_horizon,1); % initial guess of angular velocity

controls = [v0, w0];

v_list = [];
w_list = [];
trace_path = [cur_state];

obstacle = [19.8066,9.6890,1];
obs_v = 0;
obs_w = 0;
agent_radius = 0.5; % actual agent radius is 0.5. I have considered 1 to have more space for obstacle avoidance.
obstacle_radius = 1;
% Initialize video
myVideo = VideoWriter('data/mpc'); %open video file
myVideo.FrameRate = 15;  %can adjust this, 5 - 10 works well for me
open(myVideo)
f0 = figure;
k = 0;
while(norm(cur_state-goal_state)>1.5)
   controls = predict_controls(plan_horizon,cur_state,goal_state,vp,wp,controls(:,1),controls(:,2),agent_radius,obstacle_radius,obstacle,obs_v,obs_w,dt);
   for i=1:update_horizon
        if(norm(cur_state-goal_state)<=1.5)
            break;
        else
           k = k+1;
           cur_state = nonhn_update(cur_state,controls(i,1),controls(i,2),dt)
           [obstacle, obs_v, obs_w] = dynamic_obs(obstacle,k); 
           trace_path = [trace_path;cur_state];
           v_list = [v_list, controls(i,1)];
           w_list = [w_list, controls(i,2)];           
%            plot(cur_state(1),cur_state(2),'o');
           
           clf(f0)
           hold on;
           plot([init_state(1),goal_state(1)],[init_state(2),goal_state(2)],'rx')
           plot([init_state(1),goal_state(1)],[init_state(2),goal_state(2)],'--k')
           viscircles([cur_state(1),cur_state(2)], 0.5,'Color','b');
           [x_traj,y_traj] = plottraj(cur_state, controls);
           plot(x_traj,y_traj,'b.');
           viscircles([obstacle(1),obstacle(2)], 1,'Color','r');
           xlim([init_state(1)-10 goal_state(1)+10]);
           ylim([init_state(2)-10 goal_state(2)+10]);
           pause(1/60);
           frame = getframe(gcf); %get frame
           writeVideo(myVideo, frame);
           hold off;
        end
       vp = controls(update_horizon,1);
       wp = controls(update_horizon,2);
   end
end
close(myVideo)
close(f0)

f1 = figure;
plot(trace_path(:,1),trace_path(:,2), '.')
xlim([init_state(1)-10 goal_state(1)+10]);
ylim([init_state(2)-10 goal_state(2)+10]);
title("Agent's positions")
saveas(gcf,'data/Positions.png')
close(f1)

f2 = figure;
plot(v_list,'red')
ylim([0 21]);
title("Velocity vs Timesteps")
xlabel('Timesteps')
ylabel('Velocity')
saveas(gcf,'data/Vel_vs_timesteps.png')
close(f2)

f3 = figure;
plot(w_list,'blue')
ylim([-0.5 0.5]);
title("Angular Velocity vs Timesteps")
xlabel('Timesteps')
ylabel('Angular Velocity')
saveas(gcf,'data/AngVel_vs_timesteps.png')
close(f3)