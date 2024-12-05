desiredcruisespeed = 3;

aircraft = AircraftPowerModel.AircraftPowerModel(0.5);
pHorizontal = aircraft.calcHorizontalPower(desiredcruisespeed);
experimentName = 'LearningSoC';
% wayPoints = [0,0,100;10,10,100;20,20,100;30,30,100;40,40,100;50,50,100];
% wayPoints = [0,0,100;0,0,110;0,0,120;0,0,130;0,0,140];
% wayPoints = [0,0,100;0,0,110;0,0,120;0,0,130;0,0,140];

climbPower = aircraft.calcClimbPower(desiredcruisespeed);
distance = calculatedistance(wayPoints);
timeinterval = calculatetime(distance, desiredcruisespeed);
timeArray = timeinterval(1):0.1:timeinterval(2);
power = repmat(climbPower,length(timeArray),1);
EnergyRequirement.generateEnergyRequirement(experimentName, wayPoints)
% filename = 'LearningSoCfullMissionBatteryParams.mat';
% plotPowerProfilesOld(aircraft, filename)
% 
voltage = results{2};
current = results{3};
batteryTime = results{7};
power = repmat(climbPower,length(batteryTime),1);
plot(batteryTime,voltage.*current)
hold on 
plot(batteryTime,power)


function [totaldistance] = calculatedistance(waypoints)

totaldistance=0;
for i=1:length(waypoints)-1
    totaldistance=totaldistance+norm(waypoints(i+1,:)-waypoints(i,:));
end
end

function [timeinterval] = calculatetime(distance,desiredcruisespeed)
timeinterval=[0 distance/desiredcruisespeed];
end

