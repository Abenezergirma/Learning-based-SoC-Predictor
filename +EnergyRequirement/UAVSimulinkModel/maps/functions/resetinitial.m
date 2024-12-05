function [IC,battery] = resetinitial(IC,battery,State,batvar)

%vehicle state
IC.X=State(end,1);
IC.Y=State(end,2);
IC.Z=State(end,3);
IC.U=State(end,4);
IC.V=State(end,5);
IC.W=State(end,6);
IC.Phi=State(end,7);
IC.The=State(end,8);
IC.Psi=State(end,9);
IC.P=State(end,10);
IC.Q=State(end,11);
IC.R=State(end,12);
IC.state=[IC.X IC.Y IC.Z IC.U IC.V IC.W IC.Phi IC.The IC.Psi];

%motors state
IC.w1=State(end,13);
IC.w2=State(end,14);
IC.w3=State(end,15);
IC.w4=State(end,16);
IC.w5=State(end,17);
IC.w6=State(end,18);
IC.w7=State(end,19);
IC.w8=State(end,20);

%battery state
battery.x0.('Tb')=batvar(end,1);
battery.x0.('Vo')=batvar(end,2);
battery.x0.('Vsn')=batvar(end,3);
battery.x0.('Vsp')=batvar(end,4);
battery.x0.('qnB')=batvar(end,5);
battery.x0.('qnS')=batvar(end,6);
battery.x0.('qpB')=batvar(end,7);
battery.x0.('qpS')=batvar(end,8);

% IC.w1=round(State.signals.values(end,13),2);
% IC.w2=round(State.signals.values(end,14),2);
% IC.w3=round(State.signals.values(end,15),2);
% IC.w4=round(State.signals.values(end,16),2);
% IC.w5=round(State.signals.values(end,17),2);
% IC.w6=round(State.signals.values(end,18),2);
% IC.w7=round(State.signals.values(end,19),2);
% IC.w8=round(State.signals.values(end,20),2);

end

