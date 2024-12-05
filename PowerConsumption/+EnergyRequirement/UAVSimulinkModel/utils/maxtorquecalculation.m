% Max torque calculation in each axis

%% constants
% arm length - distances from the octorotor center of mass ...
%        to the center of mass of all its rotors are equal and denoted by l.
l=0.635;
% thrust motor constant [Ns^2/rad^2]
b=9.8419e-05; % 
% rotor drag constant [Nms^2/rad^2]
d=1.8503e-06; % 
%angles
angsm=cos(pi/8); % 22.5 
anglg=cos(3*pi/8); % 67.5
% Maximum angular velocity per motor [rad/s]
Max_speed=670;
% Minimum angular velocity pero motor [rad/s]
Min_speed=232;

%% Tarot T-18 Spidershape

% Max Tx
[TxXmax,TyXmax,TzXmax] = evaltorques(Max_speed,Max_speed,Min_speed,Min_speed,Min_speed,Min_speed,Max_speed,Max_speed,l,b,d,angsm,anglg);

% Max Ty
[TxYmax,TyYmax,TzYmax] = evaltorques(Min_speed,Min_speed,Min_speed,Min_speed,Max_speed,Max_speed,Max_speed,Max_speed,l,b,d,angsm,anglg);

% Max Tz
[TxZmax,TyZmax,TzZmax] = evaltorques(Max_speed,Min_speed,Max_speed,Min_speed,Max_speed,Min_speed,Max_speed,Min_speed,l,b,d,angsm,anglg);

