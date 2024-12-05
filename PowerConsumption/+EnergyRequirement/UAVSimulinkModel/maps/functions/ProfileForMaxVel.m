function [endTimes,peakVels] = ProfileForMaxVel(wpts,maxVelocity)
    %   helperProfileForMaxVel Generate parameters for a trapezoidal velocity profile so it will meet velocity constraints
    %   
    %   This function creates a fast trapezoidal profile with a specified velocity limit as an upper bound.
    %   Copyright 2021 The MathWorks, Inc.
    
    % Check that maxVelocity is a scalar
    validateattributes(maxVelocity,{'numeric'},{'scalar'});

    % Find segment lengths along each dimension of waypoints.
    segLengths = abs(diff(wpts,1,2));

    % Find minimum endTime by assigning maximum velocity to longest segment
    maxSegLengths = max(segLengths,[],1);
    endTimeLowerBound = (maxSegLengths/maxVelocity);

    % Choose acceleration by multiplying endTime to be greater than lower bound
    greaterThanFactor = 1.1;
    endTimes = repmat(greaterThanFactor*endTimeLowerBound,size(wpts,1),1);

    % Determine min and max peak velocity for each segment and dimension
    % Choose largest velocity that doesn't exceed maximum velocity
    minPeakVels = segLengths./endTimes;
    maxPeakVels = 2*segLengths./endTimes;

    peakVels = min(maxPeakVels,maxVelocity);
    peakVels = max(minPeakVels,peakVels);

    % Replace any zero-values with peak velocity
    peakVels(peakVels==0) = maxVelocity;
end

