%% Tunning of PID controller using Particle Swarm Optimization 
%
%
% Author: Zhelin Chen (zhelinchen91@gmail.com)
%


%% Initialization
clear
clc
n = 50;           % Size of the swarm " no of birds "
num_step = 50;   % Maximum number of "birds steps"
dim = 3;          % Dimension of the problem, P,I,D Gain

phi_2 =0.7;          % PSO parameter C1 
phi_1 = 0.1;        % PSO parameter C2 
w =0.5;           % pso momentum or inertia  
pso_cost = 0*ones(n,num_step); % evaluation cost function for n bird in num_step

%-----------------------------%
%    initialize the parameter %
%-----------------------------%
                                       
R1 = rand(dim, n);
R2 = rand(dim, n);
current_cost =0*ones(n,1);

%------------------------------------------------%
% Initializing swarm and velocities and position %
%------------------------------------------------%
                                 
current_position = 10*(rand(dim, n)-.5); % Randomly sample from [-5,5)
velocity = .3*randn(dim, n) ; %random action [0,0.3)
local_best_position  = current_position ;

%-------------------------------------------%
%     Evaluate initial population           %           
%-------------------------------------------%

for i = 1:n
    current_cost(i) = tracklsq(current_position(:,i));    % Evaluate each population's cost
end


local_best_cost  = current_cost;
[global_best_cost,g] = min(local_best_cost); % get gbest, and index of the bird that found gbest


global_best_position = local_best_position(:,g) ;


%-------------------%
%  VELOCITY UPDATE  %
%-------------------%

velocity = w *velocity + phi_1*(R1.*(local_best_position-current_position)) + phi_2*(R2.*(global_best_position-current_position));

%------------------%
%   SWARMUPDATE    %
%------------------%
                                               
            
current_position = current_position + velocity ;

%------------------------%
%  evaluate anew swarm   %
%------------------------%
                                               

%% Main Loop
iter = 1 ;        % Iterations’counter
while  ( iter < num_step )
    for i = 1:n
        current_cost(i) = tracklsq(current_position(:,i));    % Evaluate each population's cost

        R1 = 0.5*rand(dim, n);
        R2 = 0.5*rand(dim, n);
        velocity = w *velocity + phi_1*(R1.*(local_best_position-current_position)) + phi_2*(R2.*(global_best_position-current_position));
        
        current_position = current_position + velocity/norm(velocity);
        
        if current_cost(i) <= local_best_cost(i)
            local_best_cost(i) = current_cost(i); % If the current cost is less than local best, update local best.
            local_best_position(:,i)  = current_position(:,i) ;
            if local_best_cost(i)< global_best_cost 
                global_best_cost = local_best_cost(i);
                global_best_position = local_best_position(:,i) ;
            end
        end
    end
    sprintf('%i th iteration', iter)
    iter=iter+1;
end % end of while loop its mean the end of all step that the birds move it 
                      
% get the optimal solution
Kp=global_best_position(1)
Ki=global_best_position(2)
Kd=global_best_position(3)
global_best_cost
               


    

%          
         