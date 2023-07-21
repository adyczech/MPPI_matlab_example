clc, clear, close all

%% Param defintion
n_samples = 400;    % Number of rollout trajectories
horizon = 25;       % Prediction horizon represented as number of steps
lambda = 10;        % Temperature - Selectiveness of trajectories by their costs (The closer this value to 0, the "more" we take in considiration controls with less cost), 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories without cost consideration
nu = 500;           % Exploration variance
R = diag([1, 5]);   % Control weighting matrix
cov = [1, 0.4];     % Variance of control inputs disturbance 
dt = 0.1;           % Time step of controller and simulation

init_state = [0, 0, 0, 0, 0]; % x, y, phi, v, steer
goal_state = [6, 6, 0];

%% Define environment - obstacles [x, y, radius]
obstacles = [];
% obstacles = [1.5,1, 0.5;
%     3 4 0.5;
%     1 3 0.5;
%     4 3 0.5;
%     2.5 2.5 0.5
%     ]; % x, y, radius
% obstacles = [2,0,0.5];

n_obstacles = 40;
obstacles = [rand(n_obstacles,2)*4+1, 0.2*ones(n_obstacles,1)];

%% Init
car_real = VehicleModel();
car = VehicleModel();
controller = MPPIController(lambda, cov, nu, R, horizon, n_samples, car, dt, goal_state, obstacles);

%% Prepare visualisation
fig = figure;

hold on
axis equal
xlim([-0.5 + min(init_state(1), goal_state(1)), 0.5 + max(init_state(1), goal_state(1))]);
ylim([-0.5 + min(init_state(2), goal_state(2)), 0.5 + max(init_state(2), goal_state(2))]);
plot_state(init_state, 'bo');
plot_state(goal_state, 'ro');
plot_obstacles(obstacles);

%% Control
car_state = init_state;

for i = 1:100    
    action = controller.get_action(car_state);
    controller.plot_rollouts(fig);
    
    car_state = car_real.step(action, dt, car_state);
    plot_state(car_state, 'go')

    exportgraphics(gcf,'animation.gif','Append',true);
    pause % Step the control loop by pressing any key    
end

%% Utility functions

function plot_obstacles(obstacles) 
    for i = 1:size(obstacles,1)
        r = obstacles(i,3);
        pos = [obstacles(i,[1,2])-r 2*r 2*r];
        rectangle('Position',pos,'Curvature',[1 1], 'FaceColor', 'k', 'Edgecolor','none');
    end
end


function plot_state(state, style)
    x = state(1);
    y = state(2);
    phi = state(3);

    plot(x, y, style);
    [delta_x, delta_y] = pol2cart(phi, 0.5);
    quiver(x, y, delta_x, delta_y)
end