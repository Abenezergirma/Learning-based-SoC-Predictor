% Streamlined UAV Power Analysis Script
% This script processes 3D waypoints for either a long or short trajectory,
% segments them into climb, cruise, and descent phases, visualizes the trajectory,
% and compares power consumption between the AircraftPowerModel and the Simulink model.

% Initialize AircraftPowerModel
aircraft = AircraftPowerModel.AircraftPowerModel(0.5);

%% User Selection for Trajectory Type
trajectoryType = 'long'; % Options: 'long' or 'short'

%% Generate Waypoints Based on Selected Trajectory
initialPoint = [0, 0, 100]; % Starting point at 100m altitude
if strcmp(trajectoryType, 'long')
    climb_altitude = linspace(100, 140, 20);
    cruise_distance = linspace(0, 2000, 30);
    wayPoints = [repmat(0, 20, 1), repmat(0, 20, 1), climb_altitude';
                 cruise_distance', zeros(30, 1), repmat(140, 30, 1)];
else
    climb_altitude = linspace(100, 140, 20);
    cruise_distance = linspace(0, 1000, 30);
    x_short = cruise_distance * cosd(60);
    y_short = cruise_distance * sind(60);
    wayPoints = [repmat(0, 20, 1), repmat(0, 20, 1), climb_altitude';
                 x_short', y_short', repmat(140, 30, 1)];
end

%% Segment Waypoints into Climb, Cruise, and Descent Phases
[climb_seg, cruise_seg, descend_seg] = segmentWaypoints(wayPoints);

%% Visualize Trajectories with Segments in 3D
figure('Color', 'w', 'Position', [100, 100, 600, 400]);
hold on;
plot3(wayPoints(:,1), wayPoints(:,2), wayPoints(:,3), 'k-', 'LineWidth', 2, 'DisplayName', 'Original Trajectory');
plot3(climb_seg(:,1), climb_seg(:,2), climb_seg(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'Approx Climb');
plot3(cruise_seg(:,1), cruise_seg(:,2), cruise_seg(:,3), 'g-', 'LineWidth', 2, 'DisplayName', 'Approx Cruise');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; legend('Location', 'best');
title('3D Trajectory');
view(3);
hold off;
fileName = 'Trajectory';
saveas(gcf, [fileName, '.png']);
%% Simulate the Simulink Model
experimentName = strcat(trajectoryType, 'TrajectorySim');
EnergyRequirement.generateEnergyRequirement(experimentName, wayPoints);
load(fullfile('EnergyRequirementResults', strcat(experimentName, 'fullMissionBatteryParams.mat')));
simPowerProfile = results{2} .* results{3};
batteryTime = results{7};

%% Compute Power Consumption for Each Segment
speed = 5; % m/s
climbTimeArray = estimateClimbTime(climb_seg, speed);
cruiseTimeArray = estimateCruiseTime(cruise_seg, 4);
power_climb = computePowerProfile(climb_seg, 'climb', aircraft, speed);
power_cruise = computePowerProfile(cruise_seg, 'cruise', aircraft, speed);
power_total = [power_climb; power_cruise];

%% Plot Combined Power Profile Comparison
figure('Color', 'w', 'Position', [100, 100, 600, 300]);
hold on;
plot(batteryTime, simPowerProfile, 'k-', 'LineWidth', 2, 'DisplayName', 'Actual Model');
plot(climbTimeArray, power_climb, 'r-', 'LineWidth', 2, 'DisplayName', 'Approx Climb');
plot(cruiseTimeArray + climbTimeArray(end), power_cruise, 'g-', 'LineWidth', 2, 'DisplayName', 'Approx Cruise');
xlabel('Time (s)'); ylabel('Power (W)');
title('Combined Power Profile Comparison');
legend('Location', 'best'); grid on;
hold off;
fileName = 'PowerProfile';
saveas(gcf, [fileName, '.png']);
%% Save Power Profile Data
save(fullfile('EnergyRequirementResults', strcat(experimentName, '_PowerProfile.mat')),...
    'climbTimeArray', 'cruiseTimeArray', 'power_climb', 'power_cruise', 'power_total');

%% --- Helper Functions ---
function [climbSeg, cruiseSeg, descendSeg] = segmentWaypoints(wayPoints)
    z_diff = diff(wayPoints(:, 3));
    climbSeg = wayPoints(z_diff > 0, :);
    cruiseSeg = wayPoints(z_diff == 0, :);
    descendSeg = wayPoints(z_diff < 0, :);
end

function timeArray = estimateClimbTime(segment, climbSpeed)
    z_diff = diff(segment(:, 3));
    time_steps = abs(z_diff) / climbSpeed;
    timeArray = [0; cumsum(time_steps)];
end

function timeArray = estimateCruiseTime(segment, cruiseSpeed)
    xy_diff = sqrt(sum(diff(segment(:, 1:2)).^2, 2));
    time_steps = abs(xy_diff) / cruiseSpeed;
    timeArray = [0; cumsum(time_steps)];
end

function powerProfile = computePowerProfile(segment, phase, aircraft, speed)
    if strcmp(phase, 'climb')
        power = aircraft.calcClimbPower(speed);
    elseif strcmp(phase, 'cruise')
        power = aircraft.calcHorizontalPower(speed);
    end
    powerProfile = repmat(power, size(segment, 1), 1);
end
