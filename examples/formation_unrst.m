figure(99)
clf
% close all;
% clear all;
% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();


% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents(); 

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
N = 8;
confidence_level = 0.9; % not used by SBC
SafetyRadius = 0.2; % should manually keep consistent with the initial value in ARobotarium
r = rb.set_number_of_agents(N).set_save_data(false).build();

r.set_dynamics_flag(true);
r.set_ghost_flag(true);
% rng(99);
x_rand_span_x = 0.02*randi([3 4],1,N); % setting up position error range for each robot, rand_span serves as the upper bound of uncertainty for each of the robot
x_rand_span_y = 0.02*randi([1 4],1,N); %
v_rand_span = 0.005*ones(2,N); % setting up velocity error range for each robot

r.set_ghost_error_box([x_rand_span_x;x_rand_span_y]);
r.set_vel_error(zeros(2,N));

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_poses();
ctrl_flag = [1 10 2 20 3 30 4 0 40];

r.set_radius(SafetyRadius);
r.set_ghost_poses_error(zeros(2,N));
r.step();
r.set_ctrl_flag(ctrl_flag);



% x = [-1.3 2.4 -1.1 1.7 1.6 -1.3;...
%     0 -0.2 1.7 -1.7 1.2 -1.8;...
%     0 0 0 0 0 0]; %

x = [-1.3 2.4 -1.1 1.7 1.6 -1.3 -2.1 1.0;...
    0 -0.2 1.7 -1.7 1.2 -1.8 0.5 0.5;...
    0 0 0 0 0 0 0 0]; %


% x = rand(n, 2)-1;
% x = x * 2.4./vecnorm(Q, 2, 2);
% th = 2*pi*rand(N, 1);
% x = [3*cos(th), 3*sin(th)];
% x = [x zeros(N, 1)]';             

r.set_poses(x);
% goal_condition = x(1:2,[2 1 4 3 6 5]); % robots move to swap their positions   

x = x(:, 1:N);
% goal_condition = goal_condition(:, 1:N);
hold on;

si_to_uni_dynamics = create_si_to_uni_mapping2();
fun_rand = @(a,B) [a.*B(1,:);a.*B(2,:)];
%Get randomized initial conditions in the robotarium arena
initial_conditions = generate_initial_conditions(N, 'Width', r.boundaries(2)-r.boundaries(1)-0.1, 'Height', r.boundaries(4)-r.boundaries(3)-0.1, 'Spacing', 0.2);

controller = create_si_position_controller();
timer_to_stop = 500;
timer_count = 1;

form_controller = create_potential_controller();


record_video_flag = false;
if record_video_flag
    writerObj = VideoWriter('SBC_centralized.mp4', 'MPEG-4');
    open(writerObj);
    set(gca,'nextplot','replacechildren');
    set(gcf,'Renderer','zbuffer');
    r.set_video_obj(writerObj);
end

rng(150)

font_size = 12;

%% prepare for computing the performance-related metrics

while timer_count< timer_to_stop  %(~init_checker(x(1:2,:), goal_condition)) %
    
    if timer_count == 1
        text(-2.5, -3, 'Simple','FontSize',font_size);
        
    end
    handle_timestep = text(-0.5,-3,strcat('Time Step = ',num2str(timer_count)),'FontSize',font_size);
    
    x = r.get_poses();
    
    
    % add noise to observation
    x_observe = x;
    
    if timer_count==2
        r.set_init_bot_true_path(x(1:2,:));
        r.set_init_bot_observe_path(x_observe(1:2,:));
    elseif timer_count>2
        r.set_bot_true_path(x(1:2,:));
        r.set_bot_observe_path(x_observe(1:2,:));
    end
    

    dxi = form_controller(x_observe(1:2, :));
    

    dxu = si_to_uni_dynamics(dxi, x_observe); 
    
    r.set_velocities(1:N, dxu);%+vel_error
    
    r.step();   
   
   
    r.set_video_flag(record_video_flag);
    timer_count = timer_count + 1;
    delete(handle_timestep);
 
end
if record_video_flag
    writerObj = r.get_videoobj();
    close(writerObj);
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

