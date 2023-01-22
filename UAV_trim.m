clear
clc
close all

%Initialize z_guess
initialization =1;      %0 = start from initial guess
                        %1 = use saved values from last run
if (initialization == 0)
    Z_guess = zeros(14,1);
    Z_guess(1) = 85;
else
    load trim_values_straight_level
    Z_guess = [XStar;UStar];
end

%Solve unconstrained opitmization problem
[ZStar,f0] = fminsearch('cost_straight_level', Z_guess, ...
    optimset('TolX',1e-10,'MaxFunEvals', 10000,'MaxIter',10000))

XStar = ZStar(1:9);
UStar = ZStar(10:14);

%Verify that this satisfies constraints
XdotStar = UAV_model(XStar,UStar);
VaStar = sqrt(XStar(1)^2 + XStar(2)^2 + XStar(3)^2);
gammaStar = XStar(8) - atan2(XStar(3), XStar(1));
vStar = XStar(2);
phiStar = XStar(7);
psiStar = XStar(9);
%Optimal control trim points
save trim_values_straight_level
