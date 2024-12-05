classdef AircraftPowerModel
    properties
        eta_h = 1; % Efficiency factor for hover
        eta_c = 1;% Efficiency factor function handle for climb/descent
        eta_hor = 1;% Efficiency factor function handle for horizontal flight
        rho = 1.225;% Air density
        W = 10.66*9.81;% Total weight of the aircraft
        At = 1.5;% Sum of the n-disc actuator areas
        alpha0 = 0.07842;% Coefficient for angle-of-attack equation
        alpha1 = 1.189;% Coefficient for angle-of-attack equation
        alpha2 = -0.06359;% Coefficient for angle-of-attack equation
        alpha3 = 0.004595;% Coefficient for angle-of-attack equation
        b0 = 0.2579;%0.5;% Coefficient for horizontal flight 
        b1 = 0.1;%0.02347;% Coefficient for horizontal flight 
        b2 = 1;%0.4004;% Coefficient for horizontal flight 
        b3 = 1;%0.0136; % Coefficient for horizontal flight 

        c0 = 0.5493;%0.5;% Coefficient for climb
        c1 = -0.01917;% Coefficient for climb
        c2 = 1.127;% Coefficient for climb
        c3 = -0.02208; % Coefficient for climb

        d0 = 0.5591;%0.5;% Coefficient for climb
        d1 = -0.1106;% Coefficient for climb
        d2 = -0.03985;% Coefficient for climb
        d3 = -2.577; % Coefficient for climb

        resultsPath='/home/abenezertaye/Desktop/Research/Codes/Learning-based-SoC-Predictor/EnergyRequirementResults';
    end
    
    methods
        function obj = AircraftPowerModel(eta_h)
            obj.eta_h = eta_h;
        end
        
        function P_hover = calcHoverPower(obj)
            P_hover = obj.W^(3/2) / (obj.eta_h * sqrt(2 * obj.rho * obj.At));
        end
        
        function P_climb = calcClimbPower(obj, Vc)
            eta_climb = 0.14;%obj.calcEtaClimb(Vc);
            P_climb = obj.W / eta_climb * (Vc / 2 + sqrt(Vc^2 / 4 + obj.W / (2 * obj.rho * obj.At)));
        end
        
        function P_descent = calcDescentPower(obj, Vd)
            P_descent = obj.W / obj.eta_c(Vd) * (-Vd / 2 + sqrt(Vd^2 / 4 + obj.W / (2 * obj.rho * obj.At)));
        end
        
        function eta_climb = calcEtaClimb(obj, Vc)
            eta_climb = obj.c0 + obj.c1*cos(Vc*obj.c2) + obj.c3*sin(Vc*obj.c2);
        end
        
        function eta_hor_value = calcEtaHor(obj, Vhor)
            eta_hor_value = obj.b0 + obj.b1 * cos(Vhor * obj.b2) + obj.b3 * sin(Vhor * obj.b2);
        end
        
        % Update calcHorizontalPower to use calcEtaHor method
        function P_horizontal = calcHorizontalPower(obj, Vhor)
            v_hor = sqrt(-Vhor^2 / 2 + sqrt(Vhor^4 / 4 + (obj.W / (2 * obj.rho * obj.At))^2));
            alpha_v = obj.alpha0 + obj.alpha1 * Vhor + obj.alpha2 * Vhor^2 + obj.alpha3 * Vhor^3;%0;%pi/2;%
            eta_hor_value = obj.calcEtaHor(Vhor); % Use the new method here
            P_horizontal = obj.W / eta_hor_value * (Vhor * sin(alpha_v) + v_hor);
        end

        function plotPowerProfilesOld(obj, filename)
            load(fullfile(obj.resultsPath,filename));
            uav = 1;

            % for uav = 1:6
            [SOC,totali,voltage,postraj,refpostraj,time,timeb] = deal(results{uav,1},results{uav,2},results{uav,3}, ...
                results{uav,4},results{uav,5},results{uav,6},results{uav,7});


            powerProfile1 = totali.*voltage;
            % powerProfile2 = totali1.*voltage1;
            % Ensure column vectors
            timeb = timeb(:);
            powerProfile1 = powerProfile1(:);
            % timeb1 = timeb1(:);
            % powerProfile2 = powerProfile2(:);

            % Create Figure
            figure('Color', 'w', 'Position', [100, 100, 800, 400]);

            % Main Plot: Power Profiles
            subplot(2,1,1);
            hold on;
            plot(timeb, powerProfile1, 'LineWidth', 2, 'DisplayName', 'Profile 1', 'Color', [0 0 1]);
            % plot(timeb1, powerProfile2, 'LineWidth', 2, 'DisplayName', 'Profile 2', 'Color', [1 0 0]);
            xlabel('Time (s)');
            ylabel('Power (W)');
            title('Comparison of Power Profiles');
            legend('Location', 'Best');
            grid on;
            hold off;

            % Compute and Plot Cumulative Energy
            energyProfile1 = cumtrapz(timeb, powerProfile1);
            % energyProfile2 = cumtrapz(timeb1, powerProfile2);

            subplot(2,1,2);
            hold on;
            plot(timeb, energyProfile1, 'LineWidth', 2, 'DisplayName', 'Energy without reward', 'Color', [0 0 1]);
            % plot(timeb1, energyProfile2, 'LineWidth', 2, 'DisplayName', 'Energy with reward', 'Color', [1 0 0]);
            xlabel('Time (s)');
            ylabel('Energy (J)');
            title('Cumulative Energy Comparison');
            legend('Location', 'Best');
            grid on;

            % Display the actual accumulated energy at the last points
            text(timeb(end), energyProfile1(end), sprintf('%.2f J', energyProfile1(end)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 11, 'Color', [0 0 1]);
            % text(timeb1(end), energyProfile2(end), sprintf('%.2f J', energyProfile2(end)), 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'FontSize', 11, 'Color', [1 0 0]);
            hold off;

            filename = fullfile(obj.resultsPath,'EnergyComparison.png');
            % Use the print function to save the figure as a PNG image
            print(filename, '-dpng'); % '-dpng' specifies the format, '-r300' sets the resolution (dpi)
        end
   
    end
end
