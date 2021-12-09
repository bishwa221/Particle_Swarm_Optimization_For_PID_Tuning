function f = pid_optimize(x)
s = tf('s');

%plant = 69.49135/((0.00000015154*(s^2))+(0.00445525*s)+1);
%plant = 0.0311/((0.00000816*(s^2))+(0.000144022*s)+0.00096721);
plant = 1/(0.222866*(s^2) + 0.77067*s + 1);
kp = x(1);
ki = x(2);
kd = x(3);

controller = kp + ki/s + kd*s;

% Plotting the response. This might slow the computation
%step(feedback(plant*controller,1))

dt = 0.01;
t = 0:dt:1;

error = 1 - step(feedback(plant*controller,1),t);

% Cost/Fitness function
f = sum(t'.* (abs(error) *dt)   );  %ITAE 

