%Load the trim point 
TF = 10*60; %simulation time
temp = load('trim_values_straight_level');
XStar = temp.XStar;
UStar = temp.UStar;

%run model
sim("UAVSimulation.slx")

%Extract the data

t = ans.simX.Time;
X = ans.simX.Data;

figure;
for k = 1:9
    subplot(5,2,k)
    plot(t,X(:,k),'LineWidth',2)
    ylabel(['x_', num2str(k)])
    grid on
end

disp('finished')

