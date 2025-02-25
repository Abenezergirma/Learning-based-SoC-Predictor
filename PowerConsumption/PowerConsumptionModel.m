classdef PowerConsumptionModel
    properties
        eta_h = 1; % Efficiency factor for hover
        eta_c = 1; % Efficiency factor for climb/descent
        eta_hor = 1; % Efficiency factor for horizontal flight
        rho = 1.225; % Air density
        W = 10.66 * 9.81; % Total weight of the aircraft
        At = 1.5; % Sum of the n-disc actuator areas
        resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Learning-based-SoC-Predictor/PowerConsumption/EnergyRequirementResults';
    end

    methods
        function obj = PowerConsumptionModel(eta_h)
            obj.eta_h = eta_h;
        end

        function P_hover = calcHoverPower(obj)
            P_hover = obj.W^(3/2) / (obj.eta_h * sqrt(2 * obj.rho * obj.At));
        end

        function P_climb = calcClimbPower(obj, Vc)
            eta_climb = 0.1521; % Constant efficiency for now
            P_climb = obj.W / eta_climb * (Vc / 2 + sqrt(Vc^2 / 4 + obj.W / (2 * obj.rho * obj.At)));
        end

        function P_horizontal = calcHorizontalPower(obj, Vhor)
            v_hor = sqrt(-Vhor^2 / 2 + sqrt(Vhor^4 / 4 + (obj.W / (2 * obj.rho * obj.At))^2));
            P_horizontal = (obj.W / 0.2985) * (Vhor * sin(pi) + v_hor);
        end

        function processTrajectory(obj, trajectoryType)
            % Generate waypoints
            wayPoints = obj.generateWaypoints(trajectoryType);
            
            % Segment waypoints
            [climb_seg, cruise_seg] = obj.segmentWaypoints(wayPoints);
            
            % Visualize
            obj.visualizeTrajectory(wayPoints, climb_seg, cruise_seg);
            
            % Compute power profiles
            climbSpeed = 5; % m/s
            cruiseSpeed = 4; % m/s
            climbTimeArray = obj.estimateClimbTime(climb_seg, climbSpeed);
            cruiseTimeArray = obj.estimateCruiseTime(cruise_seg, cruiseSpeed);
            power_climb = obj.computePowerProfile(climb_seg, 'climb', climbSpeed);
            power_cruise = obj.computePowerProfile(cruise_seg, 'cruise', cruiseSpeed);
            
            % Load Simulink-generated battery power profile
            [batteryTime, simPowerProfile] = obj.loadSimulinkBatteryProfile(trajectoryType);

            % Plot and save power profile with Simulink comparison
            obj.plotPowerProfileComparison(batteryTime, simPowerProfile, climbTimeArray, cruiseTimeArray, power_climb, power_cruise);
            
            % Save data
            save(fullfile(obj.resultsPath, strcat(trajectoryType, '_PowerProfile.mat')), ...
                'climbTimeArray', 'cruiseTimeArray', 'power_climb', 'power_cruise', 'simPowerProfile', 'batteryTime');
        end

        function wayPoints = generateWaypoints(~, initialXY, finalXY, altitudeChange, speed, timeInterval)
            % Constants
            initialAltitude = 100;  % Fixed initial altitude (starting height)
            finalAltitude = initialAltitude + altitudeChange; % Adjusted final altitude

            % Extract initial and final positions
            x0 = initialXY(1);
            y0 = initialXY(2);
            xf = finalXY(1);
            yf = finalXY(2);

            % Compute total cruise distance
            total_distance = sqrt((xf - x0)^2 + (yf - y0)^2);

            % Compute number of cruise waypoints based on speed and time interval
            numCruiseWaypoints = floor(total_distance / (speed * timeInterval));

            % Generate waypoints for takeoff phase (assume a smooth altitude transition)
            takeoffAltitudes = linspace(initialAltitude, finalAltitude, 3); % Takeoff in 3 steps
            takeoffWaypoints = [repmat(x0, 3, 1), repmat(y0, 3, 1), takeoffAltitudes'];

            % Generate cruise waypoints
            cruise_x = linspace(x0, xf, numCruiseWaypoints);
            cruise_y = linspace(y0, yf, numCruiseWaypoints);
            cruise_z = repmat(finalAltitude, numCruiseWaypoints, 1); % Maintain cruise altitude

            % Combine takeoff and cruise waypoints
            wayPoints = [takeoffWaypoints; cruise_x', cruise_y', cruise_z];
        end



        function [climbSeg, cruiseSeg] = segmentWaypoints(~, wayPoints)
            z_diff = diff(wayPoints(:,3));
            climbSeg = wayPoints(z_diff > 0, :);
            cruiseSeg = wayPoints(z_diff == 0, :);
        end

        function visualizeTrajectory(~, wayPoints, climb_seg, cruise_seg)
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
            saveas(gcf, 'Trajectory.png');
        end

        function timeArray = estimateClimbTime(~, segment, climbSpeed)
            z_diff = diff(segment(:,3));
            time_steps = abs(z_diff) / climbSpeed;
            timeArray = [0; cumsum(time_steps)];
        end

        function timeArray = estimateCruiseTime(~, segment, cruiseSpeed)
            xy_diff = sqrt(sum(diff(segment(:, 1:2)).^2, 2));
            time_steps = abs(xy_diff) / cruiseSpeed;
            timeArray = [0; cumsum(time_steps)];
        end

        function powerProfile = computePowerProfile(obj, segment, phase, speed)
            if strcmp(phase, 'climb')
                power = obj.calcClimbPower(speed);
            else
                power = obj.calcHorizontalPower(speed);
            end
            powerProfile = repmat(power, size(segment, 1), 1);
        end

        function [batteryTime, simPowerProfile] = loadSimulinkBatteryProfile(obj, trajectoryType)
            % Load the simulated battery profile from Simulink
            experimentName = strcat(trajectoryType, 'TrajectorySim');
            filename = fullfile(obj.resultsPath, strcat(experimentName, 'fullMissionBatteryParams.mat'));
            load(filename, 'results'); % Load Simulink results
            simPowerProfile = results{2} .* results{3}; % Compute power profile
            batteryTime = results{7}; % Extract time vector
        end

        function plotPowerProfileComparison(~, batteryTime, simPowerProfile, climbTimeArray, cruiseTimeArray, power_climb, power_cruise)
            figure('Color', 'w', 'Position', [100, 100, 600, 300]);
            hold on;
            plot(batteryTime, simPowerProfile, 'k-', 'LineWidth', 2, 'DisplayName', 'Simulink Model');
            plot(climbTimeArray, power_climb, 'r-', 'LineWidth', 2, 'DisplayName', 'Approx Climb');
            plot(cruiseTimeArray + climbTimeArray(end), power_cruise, 'g-', 'LineWidth', 2, 'DisplayName', 'Approx Cruise');
            xlabel('Time (s)'); ylabel('Power (W)');
            title('Simulink vs. Approximate Power Profile Comparison');
            legend('Location', 'best'); grid on;
            hold off;
            saveas(gcf, 'PowerProfileComparison.png');
        end
    end
end
