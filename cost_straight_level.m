function [F0] = cost_straight_level(Z)
%Z vector as input

%% COST FUNCTION

X = Z(1:9);
U = Z(10:14);

xdot = UAV_model(X,U); %State derivative from the RCAM Model

%Intermediate variables
theta = X(8);
Va = sqrt(X(1)^2 +X(2)^2 +X(3)^2);  %Airspeed
alpha = atan2(X(3), X(1));
gam = theta - alpha;    %flight path angle gamma

%Constraints
Q = [xdot;
    Va-85;
    gam;
    X(2);
    X(7);
    X(9)];

H = diag(ones(1,14));   %Penalty parameter matrix

F0 = Q'*H*Q;

