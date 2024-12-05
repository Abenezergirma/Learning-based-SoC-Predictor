%% script to transform lat long to xy in meters

load missionplanner/waypoints1.mat ;
load missionplanner/waypointsorign1.mat ;

% transform
xyzENU = lla2enu(waypoints,waypointsorign,'flat');
xyic=xyzENU(1,:);
xyref=xyzENU(2:end,:);
xyic(3)=waypointsorign(3);
xyref(:,3)=waypoints(2:end,3);
save('missionplanner/xyic.mat','xyic');
save('missionplanner/xyref.mat','xyref');
%define initial conditions for the vehicle
IC= load('params/IC_HoverAt30mOcto.mat').IC;
IC.X=xyic(1);
IC.Y=xyic(2);
IC.Z=xyic(3);
%define initial waypoint
waypointcur=1;

%remove from future waypoints
futurexyref=xyref(:,1:2);
