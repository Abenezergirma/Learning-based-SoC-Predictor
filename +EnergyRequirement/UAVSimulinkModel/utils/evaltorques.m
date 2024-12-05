function [Tx,Ty,Tz] = evaltorques(w1,w2,w3,w4,w5,w6,w7,w8,l,b,d,angsm,anglg)
%EVALTORQUES Summary of this function goes here
%   Detailed explanation goes here
Tx=b*l*anglg*w1^2 + b*l*angsm*w2^2 + b*l*angsm*w7^2 + b*l*anglg*w8^2 ...
    -b*l*angsm*w3^2 -b*l*anglg*w4^2 -b*l*anglg*w5^2 -b*l*angsm*w6^2 ;

Ty= b*l*angsm*w5^2 + b*l*anglg*w6^2 + b*l*anglg*w7^2 + b*l*angsm*w8^2 ...
    -b*l*angsm*w1^2 -b*l*anglg*w2^2 -b*l*anglg*w3^2 -b*l*angsm*w4^2 ;

Tz= d*w1^2 + d*w3^2 + d*w5^2 + d*w7^2 ...
    -d*w2^2 -d*w4^2  -d*w6^2  -d*w8^2;
end

