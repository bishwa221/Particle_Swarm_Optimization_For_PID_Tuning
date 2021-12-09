clc %clear the command window
clear %clear workspace

%% Problem setting
lb = [0.01 0.01 0.01]; %lower bound
ub = [20 20 20]; %upper bound
%ub = [400 400 400]; %upper bound
%ub = [4000 4000 4000]; %upper bound

prob = @pid_optimize; %Fitness function or Objective

%% Algorithm parameters
Np = 50; %Population Size or number of particles
T = 100; %No. of iterations(max)
w = 0.1; %Inertia Weight
c1 = 1.2; %Acceleration coefficient
c2 = 0.12; %Acceleration coefficient


%% Particle Swarm Optimization
 f = NaN(Np,1); %vector to store the fitness function value of each particle
 
 D = length(lb); %number of decision variables or dimensions
 %P = randi(100,Np,D)*rand; 
 %v = randi(100,Np,D)*rand;
 P = repmat(lb,Np,1) + repmat((ub-lb),Np,1).*rand(Np,D); %generation of initial population(solution)
 v = repmat(lb,Np,1) + repmat((ub-lb),Np,1).*rand(Np,D); %generation of initial velocities
 
 for p = 1:Np
     f(p) = prob(P(p,:)); %evaluating fitness func value of the initial population
 end

 pbest = P; %Initializing the personal best solutions
 f_pbest = f; %Initializing the fitness of the personal best solutions
 
 
 [f_gbest,ind] = min(f_pbest); %determining the best fitness func value
 gbest = P(ind,:); %determining the best solution (particle)
 c = zeros(T,1);
 for t = 1:T
     [t,f_gbest]
     c(t) = f_gbest; %convergence
     for p = 1:Np
         %update the velocity
         v(p,:) = w*v(p,:) + c1*rand(1,D).*(pbest(p,:)-P(p,:)) + c2*rand(1,D).*(gbest - P(p,:));
         %update the population
         P(p,:) = P(p,:) + v(p,:);
         
         P(p,:) = max(P(p,:),lb); %bounding the violating variables to their lower bound
         %P(p,:) = min(P(p,:),ub); %bounding the violating variables to their upper bound
         
         f(p) = prob(P(p,:)); %determine the fitness value of this new solution(particle)
         
         if f(p) < f_pbest(p) %less than for minimization problem
             
             f_pbest(p) = f(p); %update fitness func value of the personal best solutioon
             pbest(p,:) = P(p,:); %update the personal best solution
             
             if f_pbest(p) < f_gbest
                 
                 f_gbest = f_pbest(p); %update the fitness func value of the best solution
                 gbest = pbest(p,:); %update the best solution
                 
             end
             
         end
         
     end
 end     
 
 bestfitness = f_gbest
 bestsol = gbest
 
figure,semilogy(c,'r')

xlim([0 150]);

%% close loop 
 s = tf('s');

% Define the TF of the plant DC motor
%plant = 69.49135/(0.00000015154*s^2+0.00445525*s+1);
plant = 1/(0.222866*(s^2) + 0.77067*s + 1);

% Input the optimized gain parameters
kp = gbest(1);	
ki = gbest(2);
kd = gbest(3);

% Define the TF of PID controller
controller = kp + ki/s + kd*s;

% Compute the complete TF of the system
System = feedback(plant*controller,1)

% List out the transient parameters; tr, ts, overshoot, etc.
stepinfo(System)

% Uncomment to plot the response
%step(System)