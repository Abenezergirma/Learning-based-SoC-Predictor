clear; clc; close all;

% Initialize Aircraft Power Model
aircraft = PowerConsumptionModel(0.5);

% User selects trajectory type
trajectoryType = 'short'; % or 'long'
wayPoints = aircraft.generateWaypoints(trajectoryType);

experimentName = strcat(trajectoryType, 'TrajectorySim');
EnergyRequirement.generateEnergyRequirement(experimentName, wayPoints);

% Simulation Parameters
update_interval = 5; % seconds
total_time = 300; % Simulation duration
current_index = 1; % Start from the first waypoint
current_position = wayPoints(current_index, :);

% Storage for Results
time_elapsed = 0;
projected_profiles_list = {};  % ✅ Stores projected profiles dynamically

% Run Online Assessment Loop
disp('Starting real-time power assessment...');

for t = 0:update_interval:total_time
    fprintf('Iteration %d | Current Position: (%.2f, %.2f, %.2f)\n', ...
            t/update_interval, current_position(1), current_position(2), current_position(3));

    % Get remaining waypoints from current position
    remaining_waypoints = wayPoints(current_index:end, :);
    if size(remaining_waypoints, 1) < 2
        disp('Aircraft reached the final waypoint.');
        break;
    end

    % **Compute Future Projection from Current Position**
    [climb_seg, cruise_seg] = aircraft.segmentWaypoints(remaining_waypoints);
    climbSpeed = 5; % m/s
    cruiseSpeed = 4; % m/s

    % Initialize time and power projection arrays
    climbTimeArray = [];
    cruiseTimeArray = [];
    power_climb = [];
    power_cruise = [];

    % Compute climb time and power only if the climb segment is non-empty
    if ~isempty(climb_seg)
        climbTimeArray = aircraft.estimateClimbTime(climb_seg, climbSpeed);
        power_climb = aircraft.computePowerProfile(climb_seg, 'climb', climbSpeed);
    end

    % Compute cruise time and power only if the cruise segment is non-empty
    if ~isempty(cruise_seg)
        cruiseTimeArray = aircraft.estimateCruiseTime(cruise_seg, cruiseSpeed);
        power_cruise = aircraft.computePowerProfile(cruise_seg, 'cruise', cruiseSpeed);
    end

    % **Adjust times to start from the current simulation time**
    if ~isempty(climbTimeArray)
        climbTimeArray = climbTimeArray + time_elapsed;
    end
    if ~isempty(cruiseTimeArray)
        if isempty(climbTimeArray)
            cruiseTimeArray = cruiseTimeArray + time_elapsed;  % If no climb, start from current time
        else
            cruiseTimeArray = cruiseTimeArray + climbTimeArray(end);  % Otherwise, maintain continuity
        end
    end

    % **Construct the time projection array properly**
    time_projection = time_elapsed;  % Start with current time
    if ~isempty(climbTimeArray)
        time_projection = [time_projection; climbTimeArray(2:end)];
    end
    if ~isempty(cruiseTimeArray)
        time_projection = [time_projection; cruiseTimeArray];
    end

    % **Construct the power projection array properly**
    if isempty(power_climb) && isempty(power_cruise)
        power_projection = [];
    elseif isempty(power_climb)
        power_projection = power_cruise;
    elseif isempty(power_cruise)
        power_projection = power_climb;
    else
        power_projection = [power_climb(1); power_climb(2:end); power_cruise];
    end

    % **Store projected profile dynamically**
    projected_profiles = struct( ...
        'time', time_projection, ...
        'power', power_projection ...
    );
    projected_profiles_list{end + 1} = projected_profiles;

    % **Update Aircraft Position**
    expected_speed = climbSpeed; % Default climb speed
    if current_position(3) >= max(wayPoints(:,3))
        expected_speed = cruiseSpeed; % Switch to cruise
    end

    traveled_distance = update_interval * expected_speed;
    [current_position, current_index] = updateCurrentPosition(wayPoints, current_position, traveled_distance, current_index);

    % **Update time log**
    time_elapsed = t + update_interval;

    % Debugging: Check projected time horizon
    fprintf('Max Time Projection: %.2f sec\n', max(time_projection));

    % **Plot real-time power profile**
    plotRealTimePowerProfile(projected_profiles_list);

    % Simulate real-time execution
    pause(1);
end

disp('Simulation Complete.');

% **Save projected profiles list**
save('projected_profiles.mat', 'projected_profiles_list');

%% **Function to Update Aircraft Position (Interpolated Movement)**
function [new_position, new_index] = updateCurrentPosition(wayPoints, current_position, traveled_distance, current_index)
    % Ensure new_index is assigned
    new_index = current_index;

    % Loop through waypoints starting from the current index
    while new_index < size(wayPoints, 1)
        next_waypoint = wayPoints(new_index + 1, :);
        direction_vector = next_waypoint - current_position;
        segment_length = norm(direction_vector);

        % If traveled distance is within this segment, interpolate position
        if traveled_distance <= segment_length
            new_position = current_position + (direction_vector / segment_length) * traveled_distance;
            return;
        else
            % Move to the next waypoint and reduce traveled distance
            traveled_distance = traveled_distance - segment_length;
            current_position = next_waypoint;
            new_index = new_index + 1;
        end
    end

    % If the final waypoint is reached, set position to the last waypoint
    new_position = wayPoints(end, :);
    new_index = size(wayPoints, 1);
end

%% **Function to Plot Real-Time Power Profile**
function plotRealTimePowerProfile(projected_profiles_list)
    figure(1); hold off; cla; % Clear axes instead of full figure reset
    hold on;

    % Dynamic future projections with changing color and style
    color_list = {'r', 'g', 'm', 'c', 'y'};
    style_list = {'-', '--', ':', '-.'}; 

    for i = 1:length(projected_profiles_list)
        projected_profile = projected_profiles_list{i};

        % **Skip empty projections to avoid crashes**
        if isempty(projected_profile.time) || isempty(projected_profile.power)
            fprintf('Skipping empty profile at index %d\n', i);
            continue;
        end

        % **Ensure valid profile lengths before plotting**
        if length(projected_profile.time) == length(projected_profile.power)
            color = color_list{mod(i-1, length(color_list)) + 1}; 
            style = style_list{mod(i-1, length(style_list)) + 1}; 
            plot(projected_profile.time, projected_profile.power, [color, style], ...
                 'LineWidth', 1.5, 'DisplayName', sprintf('Future Projection %d', i));
        else
            fprintf('Skipping invalid profile at index %d (time/power mismatch)\n', i);
        end
    end

    % Move legend outside plot
    xlabel('Time (s)'); ylabel('Power (W)');
    title('Real-Time Power Profile with Dynamic Future Projections');
    legend('Location', 'northeastoutside');
    grid on;

    % **Ensure xlim() only runs if there's valid data**
    if ~isempty(projected_profiles_list) && ~isempty(projected_profiles_list{end}.time)
        xlim([0, max(projected_profiles_list{end}.time) + 10]);
    end

    drawnow; % Force MATLAB to update the figure
    hold off;
end
