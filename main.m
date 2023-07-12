clc, clear, close all

%% Param defintion
n_samples = 300;
horizon = 20;
lambda = 10;  % Selectiveness of trajectories by their costs (The closer this value to 0, the "more" we take in considiration controls with less cost), 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories without cost consideration
cov = 0.5;
dt = 0.1;

init_state = [0, 0, 0, 0, 0]; % x, y, phi, v, steer
goal_state = [5, 5, 0];

%% Init
car_real = VehicleModel(init_state);
car = VehicleModel(init_state);
controller = MPPIController(lambda, cov, horizon, n_samples, car, dt, goal_state);

%% Control
fig = figure;

hold on
xlim([-1, 6]);
ylim([-1, 6]);
plot_state(init_state, 'bo');
plot_state(goal_state, 'ro');

car_state = init_state;

for i = 1:100    
    action = controller.get_action(car_state)
    controller.plot_rollouts(fig);
    
    car_state = car_real.step(action, dt, car_state);
    plot_state(car_state, 'go')
    pause
    
end

% figure
% 
% car.step([1,1],1);
% 
% plot(car.state.x, car.state.y)
% [delta_x, delta_y] = pol2cart(0.5, car.state.phi);
% quiver(car.state.x, car.state.y, car.state.x + delta_x, car.state.y + delta_y)
% 
% 
% car.state


function plot_state(state, style)
    x = state(1);
    y = state(2);
    phi = state(3);

    plot(x, y, style);
    [delta_x, delta_y] = pol2cart(phi, 0.5);
    quiver(x, y, delta_x, delta_y)
end