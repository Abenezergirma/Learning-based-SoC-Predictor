%--------------------------------------------------------------------------
%
% Position: Position vector (r [m]) from geodetic coordinates
%           (Longitude [rad], latitude [rad], altitude [m])
%
% Last modified:   2018/01/27   M. Mahooti
%
%--------------------------------------------------------------------------
function [wx,wy,wz] = latlongmeters(long, lati, hi)
d = 2R⋅sin⁻¹(sqrt[sin²((θ₂ - θ₁)/2) + cosθ₁⋅cosθ₂⋅sin²((φ₂ - φ₁)/2)])



% R_equ = 6378.137e3;      % Earth's radius [m]; WGS-84
% f     = 1/298.257223563; % Flattening; WGS-84
% e2     = f*(2-f);        % Square of eccentricity
% 
% wx=[];wy=[];wz=[];
% for i=1:length(long)
% 
%     lon=long(i);
%     lat=lati(i);
%     h=hi(i);
%     CosLat = cos(lat);       % (Co)sine of geodetic latitude
%     SinLat = sin(lat);
%     % Position vector
%     N = R_equ/sqrt(1-e2*SinLat*SinLat);
%     r(1) = (N+h)*CosLat*cos(lon);
%     r(2) = (N+h)*CosLat*sin(lon);
%     r(3) = ((1-e2)*N+h)*SinLat;
%     wx=[wx r(1)];
%     wy=[wy r(2)];
%     wz=[wz r(3)];
% 
% end