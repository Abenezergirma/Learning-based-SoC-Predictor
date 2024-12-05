clear all; close all; clc
addContainingDirAndSubDir;
%% load UAV parameters, map and trajectories
octomodel=load('params/TarotT18.mat').octomodel;
Motor=load('params/KDE4213XF-360_performancedata.mat').Motor;
battery=load('params/battery_Tattu.mat').battery;
battery.sampleTime = 0.1;
windspecs=load('params/TarotT18_windspecs.mat').windspecs;
load missionplanner/waypoints1.mat ;
load missionplanner/waypointsorign1.mat ;

%% trajectory settings/ mission plan / initial conditions

% mission velocity cruise speed, desired and max
maxleashvelocity=7; % in m/s  15 M/S is max nominal speed for the Tarot

% transform lat long to xy in meters
xyzENU = lla2enu(waypoints,waypointsorign,'flat');
xyzref=xyzENU(2:end,:);
xyzic=xyzENU(1,:);
xyzic(3)=waypointsorign(3);
save('missionplanner/xyzic.mat','xyzic');
save('missionplanner/xyzref.mat','xyzref');

%define initial conditions for the vehicle
IC= load('params/initialcond/IC_HoverAt30mOcto.mat').IC;
IC.X=xyzic(1);
IC.Y=xyzic(2);
IC.Z=xyzic(3);

%rework altitude reference based on initial position
xyzref(:,3)=xyzref(:,3)+xyzic(3);
% time required to complete the whole mission, currently using this for
% every waypoint, should refine for each waypoint
waypoints=xyzref(:,1:2);
totaldistancei = calculatedistance(waypoints);
timeinterval = calculatetime(totaldistancei,maxleashvelocity);
stoptimetotal=timeinterval(2)+0.25*timeinterval(2);

% time travel between waypoints
waypoints=xyzref;
nWayPoints = length(waypoints);

% Calculate the distance between waypoints
% currently unused, could be used to estimate max time to complete each
% waypoint trajectory
distance = zeros(1,nWayPoints);
for i = 2:nWayPoints
    distance(i) = norm(waypoints(i,1:3) - waypoints(i-1,1:3));
end


%% wind parameters
turbenabled=0; % 1=enabled, 0=disabled

%% simulate flight from waypoint to waypoint

% variables initialization
postraj=[];
veltraj=[];
atttraj=[];
rpmtraj=[];
refpostraj=[];
batvartraj=[];


for waypts=1:length(xyzref)
    if waypts==6
        see=1;
    end
    % read next waypoint
    xref=xyzref(waypts,1);
    yref=xyzref(waypts,2);
    zref=xyzref(waypts,3);

    % simulate until reaching next waypoint
    w=warning('off','all');
    sim_options = simset('SrcWorkspace', 'current');
    out=sim('dynamicsquat',[],sim_options);

    % obtain simulation data
    pos=state.Data(:,1:3);
    vel=state.Data(:,4:6);
    att=state.Data(:,7:9);
    rpm=motorsrpm.Data(:,1:8);
    refpos=refposition.Data(:,1:3);
    batvar=battery_data.Data(:,1:3);   

    % save variables
     postraj=[postraj;pos];
     veltraj=[veltraj;vel];
     atttraj=[atttraj;att];
     rpmtraj=[rpmtraj;rpm];
     refpostraj=[refpostraj;refpos];
     batvartraj=[batvartraj;batvar];

    %reset initial conditions to start from last simulation step
    [IC,battery] = resetinitial(IC,battery,state.Data,battery_variables.Data);
end

%% visualization of results- can be commented for continuous simulation purposes

%create time vector
time=0:octomodel.sampletime:(length(postraj)-1)*octomodel.sampletime;
timeb=0:battery.sampleTime:(length(batvartraj)-1)*battery.sampleTime;

% obtain current results
totali=batvartraj(:,1);
voltage=batvartraj(:,2);
SOC=batvartraj(:,3);

% 2 D trajectory visualization
figure1 = figure;
hold on;
scatter(refpostraj(:,1),refpostraj(:,2),200,'x');
scatter(postraj(:,1),postraj(:,2),25);
title('trajectory')

% altitude visualization
figurealt = figure;
hold on;
plot(time,postraj(:,1)); 
plot(time,postraj(:,2)); 
plot(time,postraj(:,3)); 
title('position vis')

% speeds
figure16 = figure;
hold on;
plot(time,veltraj); 
title('linear velocity')

% trajectory error
figure12 = figure;
hold on;
xerror=refpostraj(:,1)-postraj(:,1);
yerror=refpostraj(:,2)-postraj(:,2);
zerror=refpostraj(:,3)-postraj(:,3);
plot(time,xerror);
plot(time,yerror);
plot(time,zerror);
title('X,Y,Z error')

% motor angular velocity data
figure13 = figure;
plot(time,rpmtraj);  
title('Motor rpm data')

% attitude
figure17= figure;
hold on;
plot(time,atttraj); %roll,pitch,yaw
title('roll, pitch, yaw')

% total current data
figure14 = figure;
hold on;
plot(timeb,totali); 
title('current consumption')

% Voltage and State of charge change with time
figure15 = figure;
plot(timeb,voltage); 
title('voltage')

figure151 = figure;
plot(timeb,SOC*100); 
title('SOC')
%% Final remove the added paths for future experiments
userpath('clear');
