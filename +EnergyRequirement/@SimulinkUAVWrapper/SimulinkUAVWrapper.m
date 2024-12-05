classdef SimulinkUAVWrapper
    properties
        timeStep = 0.1;
        modelPath
        modelScript
        experimentName
        resultsPath
        totalCurrent
        BatteryParams
        SOC
        voltage
        actualPosTraj
        refPosTraj
        time
        timeb
        batterySampleTime
        uavSampleTime
    end

    methods
        function obj = SimulinkUAVWrapper(modelPath)
            % Constructor
            obj.modelPath = modelPath;
            % obj.modelScript = modelScript;
            % obj.experimentName = experimentName;
            % obj.resultsPath = resultsPath;
        end


        function runModel(obj, setWaypoints)
            % Run the Simulink model with the specified inputs
            cd(obj.modelPath);

            [row, column, dim] = size(setWaypoints);

            % Preallocate a cell array to store the results
            results = cell(dim,7);

            for i = 1:dim
                fprintf('Currently running %d-th experiment\n', i);
                wayPoint = squeeze(setWaypoints(:,:,i));
                [results{i,1},results{i,2},results{i,3},results{i,4},results{i, 5},results{i,6},results{i,7}]...
                    = runDetailedT18Model(wayPoint);
            end

            % Save the output of simulation
            cd(fileparts(mfilename('fullpath')));
            resultFileName = obj.experimentName;
            resultFilePath = fullfile(obj.resultsPath, resultFileName);
            save(resultFilePath, 'results');
        end

        function saveOutputData(obj, resultFilePath)
            % This method saves all important properties of the object for future experiments
            SOC  =  obj.SOC;
            totalCurrent = obj.totalCurrent;
            voltage = obj.voltage;
            actualPosTraj = obj.actualPosTraj;
            refPosTraj = obj.refPosTraj;
            time = obj.time;
            timeb = obj.timeb;
            batterySampleTime = obj.batterySampleTime;
            uavSampleTime = obj.uavSampleTime;
            save(resultFilePath, 'totalCurrent','voltage','SOC','actualPosTraj', ...
                'refPosTraj','time','timeb',"batterySampleTime","uavSampleTime");
        end

        function smoothedTrajectory = smoothTrajectory(obj, originalTrajectory, smoothness)
            % originalTrajectory: Nx3 matrix containing [x, y, z] positions of the UAV
            % smoothness: scalar indicating the number of points for interpolation (higher value = smoother)
            % smoothedTrajectory: Mx3 matrix containing smoothed [x, y, z] positions

            % Ensure originalTrajectory has enough points
            if size(originalTrajectory, 1) < 2
                error('original Trajectory must contain at least two points.');
            end

            % Extract x, y, z
            x = originalTrajectory(:, 1);
            y = originalTrajectory(:, 2);
            z = originalTrajectory(:, 3);

            % Create original parameterization
            tOriginal = 1:length(x);

            % Create smooth parameterization
            tSmooth = linspace(1, length(x), smoothness);

            % Spline interpolation
            xSmooth = spline(tOriginal, x, tSmooth);
            ySmooth = spline(tOriginal, y, tSmooth);
            zSmooth = spline(tOriginal, z, tSmooth);

            % Combine smoothed x, y, z
            smoothedTrajectory = [xSmooth', ySmooth', zSmooth'];
        end

        function smoothedTrajectories = smoothTrajectoryVectorized(obj, originalTrajectories, smoothness)
            % originalTrajectories: NxMx3 matrix containing [x, y, z] positions of M UAVs
            % smoothness: scalar indicating the number of points for interpolation (higher value = smoother)
            % smoothedTrajectories: NxMx3 matrix containing smoothed [x, y, z] positions for each UAV

            % Ensure originalTrajectories has enough points
            if size(originalTrajectories, 1) < 2
                error('originalTrajectories must contain at least two points.');
            end

            % Get the number of waypoints (N), the number of aircraft (M), and the dimensionality (3)
            [N, M, dim] = size(originalTrajectories);

            % Create original parameterization
            tOriginal = 1:N;

            % Create smooth parameterization
            tSmooth = linspace(1, N, smoothness);

            % Initialize the matrix to store smoothed trajectories
            smoothedTrajectories = zeros(smoothness, M, dim);

            % Loop through each UAV
            for i = 1:M
                % Extract x, y, z for the current UAV
                trajectory = originalTrajectories(:, i, :);

                % Initialize matrices for smoothed dimensions
                smoothedDimensions = zeros(smoothness, dim);

                % Spline interpolation for x, y, z
                for d = 1:dim
                    % Perform spline interpolation
                    smoothedDimensions(:, d) = spline(tOriginal, trajectory(:, d), tSmooth);
                end

                % Assign the smoothed dimensions to the output
                smoothedTrajectories(:, i, :) = smoothedDimensions;
            end
        end

        function plotTrajectoriesAndWaypoints(obj, trajectory1, waypoints1, trajectory2, waypoints2)
            % Set up figure
            figure;
            hold on;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Trajectories and Waypoints');
            grid on;

            % Initialize legend information
            h = []; % Handles for the plots
            labels = {}; % Labels for the plots

            % Function to plot each trajectory and waypoints
            function plotSet(trajectory, waypoints, trajColor, wpColor, label)
                uav = 6;
                x_traj = trajectory(:, uav,1);
                y_traj = trajectory(:, uav,2);
                z_traj = trajectory(:, uav,3)-950;

                x_waypoints = waypoints(:, uav,1);
                y_waypoints = waypoints(:, uav,2);
                z_waypoints = waypoints(:, uav, 3)-950;

                % Plot the trajectory
                h_traj = plot3(x_traj, y_traj, z_traj, trajColor, 'LineWidth', 1.5);
                % Plot the waypoints
                scatter3(x_waypoints, y_waypoints, z_waypoints, 100, wpColor, 'filled');

                % Collect the handles and labels for legend
                h(end+1) = h_traj;
                labels{end+1} = label;
            end

            % Plot the first trajectory and waypoints with label for "Traj without reward"
            plotSet(trajectory1, waypoints1, 'b-', 'ro', 'Traj without reward');

            % Plot the second trajectory and waypoints with label for "Traj with reward"
            plotSet(trajectory2, waypoints2, 'g-', 'mo', 'Traj with reward');

            % Add legend
            legend(h, labels, 'Location', 'best');

            % Adjust view and display the plot
            view(3);
            axis equal;
            hold off;

            % Set the filename for the PNG image
            filename = fullfile(obj.resultsPath,'trajectoryAndWaypoints.png');

            % Use the print function to save the figure as a PNG image
            print(filename, '-dpng'); % '-dpng' specifies the format
        end



        function plotTrajectoryAndWaypoints(obj,trajectories, waypoints)
            % Extract x, y, and z coordinates from trajectories and waypoints
            x_traj = trajectories(:, 1);
            y_traj = trajectories(:, 2);
            z_traj = trajectories(:, 3);

            x_waypoints = waypoints(:, 1);
            y_waypoints = waypoints(:, 2);
            z_waypoints = waypoints(:, 3);

            % Plot the trajectory
            figure;
            plot3(x_traj, y_traj, z_traj, 'b-', 'LineWidth', 1.5);
            hold on;

            % Plot the waypoints
            scatter3(x_waypoints, y_waypoints, z_waypoints, 100, 'ro', 'filled');


            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Trajectory and Waypoints');
            grid on;

            % Adjust view and display the plot
            view(3);
            axis equal;
            hold off;
            % Set the filename for the PNG image
            filename = fullfile(obj.resultsPath,'trajectoryAndWaypoint.png');

            % Use the print function to save the figure as a PNG image
            print(filename, '-dpng'); % '-dpng' specifies the format, '-r300' sets the resolution (dpi)
        end

        function generateBatteryParametersPlot(obj,results,uav)
            % Unpack the data
            [SOC,totali,voltage,postraj,refpostraj,time,timeb] = deal(results{uav,1},results{uav,2},results{uav,3}, ...
                results{uav,4},results{uav,5},results{uav,6},results{uav,7});
            % Create a new figure for the first subplot
            figure;

            % Subplot for SOC and voltage
            subplot(2, 1, 1);
            plot(timeb, voltage, 'r', 'LineWidth', 1.5);
            ylabel('Voltage');
            xlabel('Time (s)');
            legend('Voltage', 'Location', 'northeast');
            % yyaxis left;
            % plot(timeb, SOC*100, 'b', 'LineWidth', 1.5);
            % ylabel('SOC');
            % ylim([0, 100]);
            % yyaxis right;
            % plot(timeb, voltage, 'r', 'LineWidth', 1.5);
            % ylabel('Voltage');
            % xlabel('Time (s)');
            % legend('SOC', 'Voltage', 'Location', 'southwest');
            % legend('Totali', 'Location', 'Best'); % Adjust legend location
            title('Voltage');

            % Subplot for totali
            subplot(2, 1, 2);
            plot(timeb, totali, 'g', 'LineWidth', 1.5);
            ylabel('TotalCurrent');
            xlabel('Time (s)');
            title('TotalCurrent');
            legend('Current', 'Location', 'northeast');

            % Set the filename for the PNG image
            filename = fullfile(obj.resultsPath,'TotalCurrent_noenergyReward.png');
            print(filename, '-dpng');

            % Create a new figure for the second subplot
            figure;

            % Plot postraj and refpostraj
            plot3(postraj(:, 1), postraj(:, 2), postraj(:, 3)-950, 'b', 'LineWidth', 1.5);
            hold on;
            % plot3(refpostraj(:, 1), refpostraj(:, 2), refpostraj(:, 3)-950, 'r', 'LineWidth', 1.5);
            hold off;
            xlabel('X, m');
            ylabel('Y, m');
            zlabel('Z, m');
            legend('actualTraj', 'refTraj', 'Location', 'Best');
            title('Octo-copter Trajectory');

            % Adjust the figure layout
            axis equal;
            grid on;
            % Set the filename for the PNG image
            filename = fullfile(obj.resultsPath,'T18-Trajectories_noenergyReward.png');
            print(filename, '-dpng');
        end

    end
end