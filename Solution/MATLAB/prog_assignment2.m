%% Programming Assignment 2
%% ODE Function

function_converter = odeFunction(v_func, vars); % Converts symbolic function in Function
func_dx = @(t,x)function_converter(t,x);
tspan = timespan(1,1):0.1:timespan(1,2);
[time,state_space_matrix] = ode45(func_dx,tspan,initial_params);
%clear function_converter func_dx v_func;