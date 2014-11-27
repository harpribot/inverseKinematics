function [ parameters ] = inverseKinematics( e,parameters )
% inverse kinematics 

%% take initial theta
parameters(:,2) = [0;0;0;0];

%% few more inits
lambda = 100; 
dimension = length(e);
%%
% run the loop till the end effector is pretty close to the position
% provided
initial_position = e;
intended_position = e;
final_position = Forward_kinematics(parameters);
iter = 0;
while(sqrt((initial_position-final_position)'*(initial_position-final_position)) >1e-6)

%% find initial jacobian
Jacobian = zeros(3,length(parameters(:,1)));
joints = length(parameters(:,1));
parameters_new = parameters;

for i = 1:joints
    parameters_new(i,2) = parameters(i,2) - 0.01;
    Jacobian(:,i) =  (Forward_kinematics(parameters) - Forward_kinematics(parameters_new))/0.01;
    parameters_new = parameters;
end

%% update theta

J = Jacobian;
parameters(:,2) = parameters(:,2) + J' * ((J*J' + lambda*eye(dimension))\(intended_position -Forward_kinematics(parameters)));

%% calculate final position

initial_position = final_position;
final_position = Forward_kinematics(parameters);

%% iteration count

iter = iter +1
sqrt((intended_position-final_position)'*(intended_position-final_position))
%sqrt((initial_position-final_position)'*(initial_position-final_position))
end
parameters_theta = parameters(:,2);
end
