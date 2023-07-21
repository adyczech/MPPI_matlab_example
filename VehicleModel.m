classdef VehicleModel < handle
    %VEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m = 2000.0, 			% Mass (kg)
        I_zz = 3764.0, 			% Inertia (kg m^2)
%         h_cm = 0.3,	 			% Distance from CG to front axle (m)
        l_f = 1.53, 			% Distance from CG to front axle (m)
        l_r = 1.23, 			% Distance from CG to rear axle (m)
        C_D0 = 241, 			% Coefficient of drag
        C_D1 = 25.1, 			% Coefficient of drag
        C_alphaf = 150000.0, 		% Front tire cornering stiffness (N/rad)
        C_alphar = 280000.0, 		% Rear tire cornering stiffness (N/rad)
        mu_f = 1.5, 				% Front tire friction
        mu_r = 1.5, 				% Rear tire friction

        tau_steering = 2;       % steering time constant
        tau_velocity = 3;       % velocity time constant

        max_vel = 5;

%         prev_vel = 0;
%         prev_steer = 0;

%         state = struct('x', 0, 'y', 0, 'phi', 0);       
    end
    
    methods
        function self = VehicleModel()
        end

        function state = step(self, action, dt, state)
            x = state(1);
            y = state(2);
            phi = state(3);
            prev_vel = state(4);
            prev_steer = state(5);

            vel = action(1);
            steer = action(2);

            vel = prev_vel + dt*(vel-prev_vel)/self.tau_velocity;
            steer = prev_steer + dt*(steer-prev_steer)/self.tau_steering;

            if vel > self.max_vel
                vel = self.max_vel;
            end

            x = x + vel*dt*cos(steer + phi);
            y = y + vel*dt*sin(steer + phi);
            phi = phi + steer;    

            state = [x, y, phi, vel, steer];
        end
    end
end

