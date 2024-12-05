function generateEnergyReqiurement(experimentName, wayPoints)
% This script is used to generate battery related parameters (such as voltage, current) for the full
% flight mission
% clear all; clc
projectPath ='/home/abenezertaye/Desktop/Research/Codes/Learning-based-SoC-Predictor';


%% Define File and Path Related Properties
% Define all file and path-related properties in a structured way
% Define model path
modelPath='/home/abenezertaye/Desktop/Research/Codes/Learning-based-SoC-Predictor/+EnergyRequirement/UAVSimulinkModel';

% Create an instance of SimulinkUAVWrapper and set properties
T18Wrapper = EnergyRequirement.SimulinkUAVWrapper(modelPath);
T18Wrapper.modelScript = 'runDetailedT18Model';
T18Wrapper.experimentName = strcat(experimentName,'fullMissionBatteryParams.mat');
T18Wrapper.resultsPath='/home/abenezertaye/Desktop/Research/Codes/Learning-based-SoC-Predictor/EnergyRequirementResults';

% Construct the full path for battery parameters
T18Wrapper.BatteryParams = fullfile(T18Wrapper.resultsPath,T18Wrapper.experimentName);
%% Model Execution
% Run the Simulink model
if true
T18Wrapper.runModel(wayPoints);
end

%% Load and Visualize Results
% Load the results from the executed model and generate a plot for battery parameters
cd(projectPath);
uav = 1;
load(T18Wrapper.BatteryParams)
T18Wrapper.generateBatteryParametersPlot(results,uav)
end
