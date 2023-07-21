classdef MPPIController < handle
    %MPPICONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lambda          % Temperature
        horizon
        n_samples
        cov        
        R               % Control weighting matrix
        nu              % Exploration variance

        model
        dt

        obstacles

        control_sequence
        optimized_control_sequence

        goal
        state
        n_states = 5;

        rollouts_states
        rollouts_costs
        rollouts_plot_handle = [];

        max_vel = 5;
        max_steer = 0.5; 
    end
    
    methods
        function self = MPPIController(lambda, cov, nu, R, horizon, n_samples, model, dt, goal, obstacles)
            self.lambda = lambda;
            self.horizon = horizon;
            self.n_samples = n_samples;
            self.cov = cov;
            self.nu = nu;
            self.R = R;
            self.model = model;
            self.dt = dt;
            self.goal = goal;

            self.obstacles = obstacles;

            self.control_sequence = zeros(2,self.horizon);            
        end
        
        function action = get_action(self, state)
            % Init variables
            self.state = state;
            init_state = state;
            states = zeros(self.n_states, self.horizon+1);
            S = zeros(self.n_samples, 1);

            self.rollouts_states = zeros(self.n_samples, self.horizon+1, self.n_states);
            self.rollouts_costs = zeros(self.n_samples,1);

            % Generate random control input disturbances
            delta_u_vel = normrnd(0, self.cov(1), [self.n_samples, self.horizon]);
            delta_u_steer = normrnd(0, self.cov(2), [self.n_samples, self.horizon]);

            delta_u_steer(delta_u_steer > 0.5) = 0.5;
            delta_u_steer(delta_u_steer < -0.5) = -0.5;

            delta_u(1,:,:) = delta_u_vel;
            delta_u(2,:,:) = delta_u_steer;           

            for k = 1:self.n_samples
                states(:,1) = init_state;
                for i = 1:self.horizon                    
                    % Single trajectory step
                    states(:,i+1) = self.model.step(self.control_sequence(:,i) + delta_u(:, k, i), self.dt, states(:,i));

                    % Compute cost of the state
                    S(k) = S(k) + self.cost_function(states(:,i+1), self.control_sequence(:,i), delta_u(:, k, i));
%                     fprintf("Cost: %f\n", self.cost_function(states(:,i+1)));

                end 
%                 fprintf("Actions for %d rollout: \n", k);
%                 self.control_sequence() + reshape(delta_u(:, k, :), 2,self.horizon)

%                 fprintf("States for %d trajectory: \n", k);

%                 fprintf("Costs for %d trajectory: \n", k);
                self.rollouts_states(k,:,:) = states';
                self.rollouts_costs(k) = S(k,end);
            end     

            % Update the control input according to the expectation over K sample trajectories
            S_normalized = S - min(S);
            for i = 1:self.horizon
                self.control_sequence(:,i) = self.control_sequence(:,i) + self.total_entropy(delta_u(:,:,i)', S_normalized(:))'; 
            end
            % Output saturation
            self.control_sequence(1, self.control_sequence(1,:) > self.max_vel) = self.max_vel;
            self.control_sequence(1, self.control_sequence(1,:) < -self.max_vel) = -self.max_vel;

            self.control_sequence(2, self.control_sequence(2,:) > self.max_steer) = self.max_steer;
            self.control_sequence(2, self.control_sequence(2,:) < -self.max_steer) = -self.max_steer;

            % Select control action
            self.optimized_control_sequence = self.control_sequence;

            action = self.control_sequence(:,1);
            self.control_sequence = [self.control_sequence(:,2:end), [0; 0]];
        end

        function cost = cost_function(self, state, u, du)
           state_cost = self.state_cost_function(state);
           control_cost = self.control_cost_function(u, du);

           cost = state_cost + control_cost;
        end

        function cost = state_cost_function(self, state)
            obstacle_cost = self.obstacle_cost_function(state);
            heading_cost = self.heading_cost_function(state);
            distance_cost = self.distance_cost_function(state);%norm(self.goal(1:2) - state(1:2))^2*5;

            cost = distance_cost + heading_cost + obstacle_cost;     
        end

        function cost = distance_cost_function(self, state)
            weight = 100;
%             cost = 100*norm(self.goal(1:2) - state(1:2))^2;
            cost = weight*(self.goal(1:2) - state(1:2)')*(self.goal(1:2) - state(1:2)')';
        end

        function cost = heading_cost_function(self, state)
            weight = 50;
            pow = 2;
            cost = weight*abs(self.get_angle_diff(self.goal(3),state(3)))^pow;
        end
        
        function cost = control_cost_function(self, u, du)
            cost = (1-1/self.nu)/2 * du'*self.R*du + u'*self.R*du + 1/2*u'*self.R*u;
        end

        function [obstacle_cost] = obstacle_cost_function(self, state)
            if isempty(self.obstacles)
                obstacle_cost = 0;
                return
            end
            distance_to_obstacle = sqrt(sum((state(1:2)' - self.obstacles(:,[1,2])).^2,2));
            [min_dist, min_dist_idx] = min(distance_to_obstacle);
            
            if min_dist <= self.obstacles(min_dist_idx,3)
                hit = 1;
            else
                hit = 0;
            end

            obstacle_cost = 550*exp(-min_dist/5) + 1e6*hit;
        end

        function value = total_entropy(self, du, trajectory_cost)
            exponents = exp(-1/self.lambda * trajectory_cost);

            value = sum(exponents.*du ./ sum(exponents),1);
        end

        function plot_rollouts(self, fig)
            if ~isempty(self.rollouts_plot_handle)
                for i = 1:length(self.rollouts_plot_handle)
                    delete(self.rollouts_plot_handle(i));
                end
                self.rollouts_plot_handle = [];
            end
            figure(fig)
            costs = (self.rollouts_costs - min(self.rollouts_costs))/(max(self.rollouts_costs) - min(self.rollouts_costs));
            [~, min_idx] = min(costs);
            for i = 1:self.n_samples
                if i == min_idx
                    color = [0, 1, 1];
                else
                    color = [1-costs(i), 0, 0.2];
                end
                self.rollouts_plot_handle(end+1) = plot(self.rollouts_states(i,:,1), self.rollouts_states(i,:,2),'-', 'Color', color);
%                 self.rollouts_plot_handle(end+1) = text(self.rollouts_states(i,end,1), self.rollouts_states(i,end,2), string(self.rollouts_costs(i)));
            end

            % Rollout of selected trajectory
            states = zeros(self.n_states, self.horizon+1);
            states(:,1) = self.state;

            for i = 1:self.horizon                    
                % Single trajectory step
                states(:,i+1) = self.model.step(self.optimized_control_sequence(:,i), self.dt, states(:,i));
            end
            self.rollouts_plot_handle(end+1) = plot(states(1,:), states(2,:), '--', 'Color', [0,1,0]);

        end     
    end

    methods(Static)
        function angle = get_angle_diff(angle1, angle2)
            angle_diff = angle1-angle2;
            angle = mod(angle_diff+pi, 2*pi) - pi;              
        end
    end

end

