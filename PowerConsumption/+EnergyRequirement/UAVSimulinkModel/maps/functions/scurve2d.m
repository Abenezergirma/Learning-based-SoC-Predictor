% S Curve Generator
% Implementation of Section 3.4 from 
%  Trajectory Planning for Automatic Machines and Robots (Biagiotti, Melchiorri)

function output = scurve2d(startpt, endpt, startvel, endvel, vmax, amax, jmax, Ts)

    % Define the waypoints
    x1 = startpt(1);
    y1 = startpt(2);
    x2 = endpt(1);
    y2 = endpt(2);

    % Calculate the direction vector
    D = [x2 - x1, y2 - y1];

    % Calculate the magnitude of the direction vector
    MagnitudeD = norm(D);

    q0 = 0;
    q1 = MagnitudeD;
    v0 = norm(startvel);
    v1 = norm(endvel);
    result = continuousSCurve(q0, q1, v0, v1, vmax, amax, jmax, Ts);
    p = result(:,1);
    v = result(:,2);
    a = result(:,3);
    j = result(:,4);

    % figure;
    % plot(v);
    % title('Velocity');
    % 
    % figure;
    % plot(j);
    % title('Jerk');
    % 
    % figure;
    % plot(a);
    % title('Acceleration');
    % 
    % figure;
    % plot(p);
    % title('Position');
    
    % Normalize the direction vector
    D_normalized = D / MagnitudeD;
    
    % Convert scalar velocities to vector velocities
    X_vel = zeros(numel(v), 1);
    Y_vel = zeros(numel(v), 1);
    for j = 1:numel(v)
        X_vel(j) = v(j) * D_normalized(1);
        Y_vel(j) = v(j) * D_normalized(2);
    end

    output = [X_vel, Y_vel];

    % Function to determine if a trajectory is possible (not currently
    % used)
    function output = isTrajectoryPossible(q0, q1, v0, v1, amax, jmax)
        lhs = sqrt(abs(v1-v0)/jmax);
        rhs = amax/jmax;
    
        T_starj = min([lhs, rhs]);
    
        diff = q1 - q0;
    
        if T_starj < (amax/jmax) 
            cond = T_starj * (v0 + v1);
        elseif T_starj == (amax/jmax)
            cond = 0.5*(v0+v1)*(T_starj+(abs(v1-v0)/amax));
        else
            output = false;
            return;
        end 
    
        output = diff > cond;
    end

    % Generates a Double S velocity profile
    function output = continuousSCurve(q0, q1, v0, v1, vmax, amax, jmax, Ts)
        jmin = -1 * jmax;

        % Computes the time cutoffs for each of the phases
        times = computeTimeEdgeCases(q0, q1, v0, v1, vmax, amax, jmax);
        Ta = times(1);
        Tv = times(2);
        Td = times(3);
        Tj1 = times(4);
        Tj2 = times(5);
        T = Ta + Tv + Td + Tj1 + Tj2;

        alima = jmax * Tj1;
        alimd = -1 * jmax * Tj2;
        vlim = v0 + (Ta - Tj1) * alima;

        % disp([Ta, Tv, Td, Tj1, Tj2]);
        % disp([alima, alimd, vlim]);

        positions = [];
        vels = [];
        accs = [];
        jerks = [];

        % Acceleration Phase 

        % a)
        startt = 0;
        endt = floor(Tj1 / Ts);

        for i = startt:endt
            t = i * Ts;
            newpos = q0 + v0 * t + jmax * (t^3/6);
            newvel = v0 + jmax * (t^2/2);
            newacc = jmax * t;
            newjerk = jmax;

            positions = [positions; newpos];
            vels = [vels; newvel];
            accs = [accs; newacc];
            jerks = [jerks; newjerk];
        end

        % b)
        startt = ceil(Tj1 / Ts);
        endt = floor((Ta - Tj1) / Ts);

        for i = startt:endt
            t = i * Ts;
            newpos = q0 + v0 * t + (alima/6) * (3*t^2 - 3*Tj1*t + Tj1^2);
            newvel = v0 + alima * (t - (Tj1/2));
            newacc = alima;
            newjerk = 0;

            positions = [positions; newpos];
            vels = [vels; newvel];
            accs = [accs; newacc];
            jerks = [jerks; newjerk];
        end

        % c)
        startt = ceil((Ta - Tj1) / Ts);
        endt = floor(Ta / Ts);

        for i = startt:endt
            t = i * Ts;
            newpos = q0 + (vlim + v0) * (Ta/2) - vlim * (Ta - t) - jmin * ((Ta - t)^3) / 6;
            newvel = vlim + jmin * ((Ta - t)^2)/2;
            newacc = -1 * jmin * (Ta - t);
            newjerk = jmin;

            positions = [positions; newpos];
            vels = [vels; newvel];
            accs = [accs; newacc];
            jerks = [jerks; newjerk];
        end

        % Constant velocity phase
        startt = ceil(Ta / Ts);
        endt = floor((Ta + Tv)/ Ts);

        for i = startt:endt
            t = i * Ts;
            newpos = q0 + (vlim + v0) * (Ta / 2) + vlim * (t - Ta);
            newvel = vlim;
            newacc = 0;
            newjerk = 0;

            positions = [positions; newpos];
            vels = [vels; newvel];
            accs = [accs; newacc];
            jerks = [jerks; newjerk];
        end

        % Deceleration phase

        %  a)
        startt = ceil((T - Td) / Ts);
        endt = floor((T - Td + Tj2)/ Ts);

        for i = startt:endt
            t = i * Ts;
            newpos = q1 - (vlim + v1) * (Td/2) + vlim * (t - T + Td) - jmax * ((t - T + Td)^3)/6;
            newvel = vlim - jmax * ((t - T + Td)^2)/2;
            newacc = -1 * jmax * (t - T + Td);
            newjerk = -1 * jmax;

            positions = [positions; newpos];
            vels = [vels; newvel];
            accs = [accs; newacc];
            jerks = [jerks; newjerk];
        end

        %  b)
        startt = ceil((T - Td + Tj2)/ Ts);
        endt = floor((T - Tj2)/ Ts);

        for i = startt:endt
            t = i * Ts;
            newpos = q1 - (vlim + v1) * (Td/2) + vlim * (t - T + Td) + (alimd/6) * (3 * (t - T + Td)^2 - 3 * Tj2 * (t - T + Td) + Tj2^2);
            newvel = vlim + alimd * (t - T + Td - (Tj2/2));
            newacc = alimd;
            newjerk = 0;

            positions = [positions; newpos];
            vels = [vels; newvel];
            accs = [accs; newacc];
            jerks = [jerks; newjerk];
        end

        %  c)
        startt = ceil((T - Tj2)/ Ts);
        endt = floor(T / Ts);

        for i = startt:endt
            t = i * Ts;
            newpos = q1 - v1 * (T - t) - jmax * ((T-t)^3)/6;
            newvel = v1 + jmax * ((T-t)^2)/2;
            newacc = -1 * jmax * (T - t);
            newjerk = jmax;

            positions = [positions; newpos];
            vels = [vels; newvel];
            accs = [accs; newacc];
            jerks = [jerks; newjerk];
        end

        output = [positions, vels, accs, jerks];

    end

    % Wrapper function tha handles potential edge cases with computing the
    % time
    function output = computeTimeEdgeCases(q0, q1, v0, v1, vmax, amax, jmax)
        times = computeTimes(q0, q1, v0, v1, vmax, amax, jmax);
        Ta = times(1);
        Tv = times(2);
        Td = times(3);
        Tj1 = times(4);
        Tj2 = times(5);
        gamma = 1;
        while (Ta < 2 * Tj1 || Td < 2 * Tj2) && Ta >= 0 && Td >= 0
            gamma = gamma - 0.01;
            times = computeTimes(q0, q1, v0, v1, vmax, amax*gamma, jmax);
            Ta = times(1);
            Tv = times(2);
            Td = times(3);
            Tj1 = times(4);
            Tj2 = times(5);

            if Ta < 0
                Ta = 0;
                Tv = 0;
                Tj1 = 0;
                Td = 2 * (q1 - q0) / (v1 + v0);
                Tj2 = (jmax * (q1 - q0) - sqrt(jmax * (jmax * (q1 - q0)^2 + (v1 + v0)^2 * (v1 - v0)))) / (jmax * (v1 + v0));
            end

            if Td < 0
                Td = 0;
                Tv = 0;
                Tj2 = 0;
                Ta = 2 * (q1 - q0) / (v1 + v0);
                Tj1 = (jmax * (q1 - q0) - sqrt(jmax * (jmax * (q1 - q0)^2 - (v1 + v0)^2 * (v1 - v0)))) / (jmax * (v1 + v0));
            end
        end

        output = [Ta, Tv, Td, Tj1, Tj2];
    end

    function output = computeTimes(q0, q1, v0, v1, vmax, amax, jmax)
        reachedamax = amaxReached(vmax, v0, jmax, amax);
        reachedamin = aminReached(vmax, v1, jmax, amax);

        if ~reachedamax
            Tj1 = sqrt((vmax - v0) / jmax);
            Ta = 2 * Tj1;
        else
            Tj1 = amax / jmax;
            Ta = Tj1 + (vmax - v0) / amax;
        end

        if ~reachedamin
            Tj2 = sqrt((vmax - v1)/jmax);
            Td = 2 * Tj2;
        else
            Tj2 = amax / jmax;
            Td = Tj2 + (vmax - v1) / amax;
        end

        Tv = (q1 - q0) / vmax - 0.5 * Ta * (1 + v0/vmax) - 0.5 * Td * (1 + v1/vmax);

        if Tv < 0
            Tv = 0;
            Tj1 = amax/jmax;
            Tj2 = amax/jmax;
            Tj = amax/jmax;

            Delta = ((amax^4)/(jmax^2)) + 2 * (v0^2 + v1^2) + amax * (4*(q1 - q0) - 2 * (amax/jmax * (v0 + v1)));

            Ta = (((amax^2)/jmax) - 2 * v0 + sqrt(Delta)) / ( 2 * amax);
            Td = (((amax^2)/jmax) - 2 * v1 + sqrt(Delta)) / ( 2 * amax);
        end

        output = [Ta, Tv, Td, Tj1, Tj2];
    end

    function output = amaxReached(vmax, v0, jmax, amax)
        output = ~((vmax - v0) * jmax < amax^2);
    end

    function output = aminReached(vmax, v1, jmax, amax)
        output = ~((vmax - v1) * jmax < amax^2);
    end

    

end