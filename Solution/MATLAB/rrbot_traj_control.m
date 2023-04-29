clc;
clear all;
close all;

% ROS Setup
rosinit;
syms t0 efforts t
syms theta1 theta2_dt theta1_dot2
syms theta2 theta2_dt theta2_dot2

m1 = 1; m2 = 1; l1 = 1; l2 = 1; r1 = 0.45; r2 = 0.45;
I1 = 0.084; I2 = 0.084; g = 9.81;

j1_effort = rospublisher("/rrbot/joint1_effort_controller/command");
j2_effort = rospublisher("/rrbot/joint2_effort_controller/command");

JointStates = rossubscriber("/rrbot/joint_states");

tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);

tau1.Data = 0;
tau2.Data = 0;

send(j1_effort, tau1);
send(j2_effort, tau2);

client = rossvcclient("/gazebo/set_model_configuration");
req = rosmessage(client);
req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1', 'joint2'};
req.JointPositions = [deg2rad(200), deg2rad(125)];
resp = call(client, req, 'Timeout', 3);

tic;
t = 0;
i = 1;

while (t < 10)
    t = toc;
    x1d = (pi * t^3) / 500 - (3 * pi * t^2) / 100 + pi;
    dx1d = (3 * pi * t^2) / 500 - (3 * pi * t) / 50;
    v1d = (3 * pi * t) / 250 - (3 * pi) / 50;

    x2d = (pi * t^3) / 1000 - (3 * pi * t^2) / 200 + pi / 2;
    dx2d = (3 * pi * t^2) / 1000 - (3 * pi * t) / 100;
    v2d = (3 * pi * t) / 500 - (3 * pi) / 100;

    % read the joint states
    jointData = receive(JointStates);
    X = [(jointData.Position(1)); (jointData.Position(2)); jointData.Velocity(1); jointData.Velocity(2)];
    joint_states(i,:) = [jointData.Position(1);jointData.Position(2);jointData.Velocity(1);jointData.Velocity(2)]; % % X(4)

    K =  [30 0 11 0;  0 12 0 7 ];
    theta1 = X(1);
    theta2 = X(2);
    theta1_dt = X(3);
    theta2_dt = X(4);

    des_X = [jointData.Position(1) - x1d; jointData.Position(2) - x2d; jointData.Velocity(1) - dx1d; jointData.Velocity(2) - dx2d];

    M = [(9*cos(theta2))/10 + sym(1573/1000), (9*cos(theta2))/20 + sym(573/2000); (9*cos(theta2))/20 + sym(573/2000), sym(573/2000)];
    C = [-(theta2_dt*sin(theta2)*((9*theta2_dt)/10 + (9*theta1_dt)/5))/2; (9*theta1_dt^2*sin(theta2))/20];
    G = [- (8829*sin(theta2 + theta1))/2000 - (28449*sin(theta1))/2000; -(8829*sin(theta2 + theta1))/2000];
    
    v = [-K(1, :) * des_X + v1d; -K(2, :) * des_X + v2d];
    tau = double(M * v + C + G);

    tau1.Data = tau(1,1);
    tau2.Data = tau(2,1);

    send(j1_effort, tau1);
    send(j2_effort, tau2);

    efforts(i,1) = tau(1,1);
    efforts(i,2) = tau(2,1);
    virtual_input(i,:) = double(v)';
    time(i) = t;
    i = i + 1;
end

tau1.Data = 0;
tau2.Data = 0;

send(j1_effort, tau1);
send(j2_effort, tau2);
%% Plot Graphs

plot(time, joint_states(:, 1), 'blue')
legend('Actual Value', 'Trajectory', 'Error')
xlabel('t')
ylabel('${\theta_1}$', 'Interpreter', 'latex')
title('Gazebo Simulation')
grid on;
saveas(gcf, 'gazebo_theta1.jpg')

plot(time, joint_states(:, 2), 'blue')
legend('Actual Value', 'Trajectory', 'Error')
xlabel('t')
ylabel('$\dot{\theta_1}$', 'Interpreter', 'latex')
title('Gazebo Simulation')
grid on;
saveas(gcf, 'gazebo_dtheta1.jpg')

plot(time, efforts(:, 1), 'black')
hold on;
plot(time, virtual_input(:, 1), 'red')
hold off;
legend('Torque','Virtual Input')
grid on;
xlabel('time');
ylabel('\tau_{1}')
saveas(gcf, 'gazebo_tau1.jpg')

plot(time, joint_states(:, 3), 'blue')
legend('Actual Value', 'Trajectory', 'Error')
xlabel('t')
ylabel('${\theta_2}$', 'Interpreter', 'latex')
title('Gazebo Simulation')
grid on;
saveas(gcf, 'gazebo_theta2.jpg')

plot(time, joint_states(:, 4), 'blue')
legend('Actual Value', 'Trajectory', 'Error')
xlabel('t')
ylabel('$\dot{\theta_2}$', 'Interpreter', 'latex')
title('Gazebo Simulation')
grid on;
saveas(gcf, 'gazebo_dtheta2.jpg')

plot(time, efforts(:, 2), 'black')
hold on;
plot(time, virtual_input(:, 2), 'red')
hold off;
legend('Torque','Virtual Input')
grid on;
xlabel('time');
ylabel('\tau_{2}')
saveas(gcf, 'gazebo_tau2.jpg')
%disconnect from roscore
rosshutdown;
%% Store Values

delete 'gazebo_efforts.xls';
delete 'gazebo_theta_plot_points.xls';
delete 'gazebo_virtual_input.xls';
delete 'gazebo_time.xls';

writematrix(double(efforts),'gazebo_efforts.xls');
writematrix(joint_states,'gazebo_theta_plot_points.xls');
writematrix(virtual_input,'gazebo_virtual_input.xls');
writematrix(time','gazebo_time.xls');