function batteryModel_Sfcn(block)
% Level-2 MATLAB file S-Function for inherited sample time demo.
%   Copyright 1990-2009 The MathWorks, Inc.

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 4;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode      = 'Sample';
  
  for i = 1:3
    block.OutputPort(i).Dimensions       = 1;
    block.OutputPort(i).SamplingMode     = 'Sample';
  end

  block.OutputPort(4).Dimensions       = 8;
  block.OutputPort(4).SamplingMode     = 'Sample';
  
  block.NumDialogPrms     = 1;
  
  %% Set block sample time to inherited
%    block.SampleTimes = [-1 0];
  dt = block.DialogPrm(1).Data.sampleTime; % 1 second by default
  block.SampleTimes = [dt 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Update',                  @Update);  
  
%endfunction

function DoPostPropSetup(block)

  %% Setup Dwork
  block.NumDworks = 1;
  block.Dwork(1).Name = 'x'; 
  block.Dwork(1).Dimensions      = 8;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
    %% Initialize Dwork
    % Initialize 8 states and outputs based on parameters
    P = block.DialogPrm(1).Data;
    % States    
    states = [P.x0.('Tb'), P.x0.('Vo'), P.x0.('Vsn'), P.x0.('Vsp'), P.x0.('qnB'),...
        P.x0.('qnS'), P.x0.('qpB'), P.x0.('qpS')];

    for i=1:8
        block.Dwork(1).Data(i) = states(i);
    end
    
    block.OutputPort(1).Data = P.n.('Vm'); % voltage 
    block.OutputPort(2).Data = P.x0.('qnS')./(0.6*P.('qSMax')); % SOC
    block.OutputPort(3).Data = P.x0.('Tb')-273.15; % Temperature (Celsius)
    %states
    for i=1:8
        block.OutputPort(4).Data(i) =states(i);
    end
  
%endfunction

function Output(block)
    % Block arguments
    parameters = block.DialogPrm(1).Data;
            
    % Extract states
    Tb = block.Dwork(1).Data(1);
    Vo = block.Dwork(1).Data(2);
    Vsn = block.Dwork(1).Data(3);
    Vsp = block.Dwork(1).Data(4);
    qnB = block.Dwork(1).Data(5);
    qnS = block.Dwork(1).Data(6);
    qpB = block.Dwork(1).Data(7);
    qpS = block.Dwork(1).Data(8);
    
    % Constraints
    Tbm = Tb-273.15;
    xpS = qpS./parameters.qSMax;
    Vep3 = parameters.Ap3.*((2.*xpS-1).^(3+1) - (2.*xpS.*3.*(1-xpS))./(2.*xpS-1).^(1-3))./parameters.F;
    Vep8 = parameters.Ap8.*((2.*xpS-1).^(8+1) - (2.*xpS.*8.*(1-xpS))./(2.*xpS-1).^(1-8))./parameters.F;
    Vep6 = parameters.Ap6.*((2.*xpS-1).^(6+1) - (2.*xpS.*6.*(1-xpS))./(2.*xpS-1).^(1-6))./parameters.F;
    Vep5 = parameters.Ap5.*((2.*xpS-1).^(5+1) - (2.*xpS.*5.*(1-xpS))./(2.*xpS-1).^(1-5))./parameters.F;
    Vep10 = parameters.Ap10.*((2.*xpS-1).^(10+1) - (2.*xpS.*10.*(1-xpS))./(2.*xpS-1).^(1-10))./parameters.F;
    Vep9 = parameters.Ap9.*((2.*xpS-1).^(9+1) - (2.*xpS.*9.*(1-xpS))./(2.*xpS-1).^(1-9))./parameters.F;
    Vep12 = parameters.Ap12.*((2.*xpS-1).^(12+1) - (2.*xpS.*12.*(1-xpS))./(2.*xpS-1).^(1-12))./parameters.F;
    Vep4 = parameters.Ap4.*((2.*xpS-1).^(4+1) - (2.*xpS.*4.*(1-xpS))./(2.*xpS-1).^(1-4))./parameters.F;
    Vep11 = parameters.Ap11.*((2.*xpS-1).^(11+1) - (2.*xpS.*11.*(1-xpS))./(2.*xpS-1).^(1-11))./parameters.F;
    Vep2 = parameters.Ap2.*((2.*xpS-1).^(2+1) - (2.*xpS.*2.*(1-xpS))./(2.*xpS-1).^(1-2))./parameters.F;
    Vep7 = parameters.Ap7.*((2.*xpS-1).^(7+1) - (2.*xpS.*7.*(1-xpS))./(2.*xpS-1).^(1-7))./parameters.F;
    Vep0 = parameters.Ap0.*((2.*xpS-1).^(0+1))./parameters.F;
    Vep1 = parameters.Ap1.*((2.*xpS-1).^(1+1) - (2.*xpS.*1.*(1-xpS))./(2.*xpS-1).^(1-1))./parameters.F;
    xnS = qnS./parameters.qSMax;
    Ven5 = parameters.An5.*((2.*xnS-1).^(5+1) - (2.*xnS.*5.*(1-xnS))./(2.*xnS-1).^(1-5))./parameters.F;
    Ven1 = parameters.An1.*((2.*xnS-1).^(1+1) - (2.*xnS.*1.*(1-xnS))./(2.*xnS-1).^(1-1))./parameters.F;
    Ven10 = parameters.An10.*((2.*xnS-1).^(10+1) - (2.*xnS.*10.*(1-xnS))./(2.*xnS-1).^(1-10))./parameters.F;
    Ven7 = parameters.An7.*((2.*xnS-1).^(7+1) - (2.*xnS.*7.*(1-xnS))./(2.*xnS-1).^(1-7))./parameters.F;
    Ven2 = parameters.An2.*((2.*xnS-1).^(2+1) - (2.*xnS.*2.*(1-xnS))./(2.*xnS-1).^(1-2))./parameters.F;
    Ven8 = parameters.An8.*((2.*xnS-1).^(8+1) - (2.*xnS.*8.*(1-xnS))./(2.*xnS-1).^(1-8))./parameters.F;
    Ven4 = parameters.An4.*((2.*xnS-1).^(4+1) - (2.*xnS.*4.*(1-xnS))./(2.*xnS-1).^(1-4))./parameters.F;
    Ven3 = parameters.An3.*((2.*xnS-1).^(3+1) - (2.*xnS.*3.*(1-xnS))./(2.*xnS-1).^(1-3))./parameters.F;
    Vep = parameters.U0p + parameters.R.*Tb./parameters.F.*log((1-xpS)./xpS) + Vep0 + Vep1 + Vep2 + Vep3 + Vep4 + Vep5 + Vep6 + Vep7 + Vep8 + Vep9 + Vep10 + Vep11 + Vep12;
    Ven0 = parameters.An0.*((2.*xnS-1).^(0+1))./parameters.F;
    Ven11 = parameters.An11.*((2.*xnS-1).^(11+1) - (2.*xnS.*11.*(1-xnS))./(2.*xnS-1).^(1-11))./parameters.F;
    Ven12 = parameters.An12.*((2.*xnS-1).^(12+1) - (2.*xnS.*12.*(1-xnS))./(2.*xnS-1).^(1-12))./parameters.F;
    Ven6 = parameters.An6.*((2.*xnS-1).^(6+1) - (2.*xnS.*6.*(1-xnS))./(2.*xnS-1).^(1-6))./parameters.F;
    Ven9 = parameters.An9.*((2.*xnS-1).^(9+1) - (2.*xnS.*9.*(1-xnS))./(2.*xnS-1).^(1-9))./parameters.F;
    Ven = parameters.U0n + parameters.R.*Tb./parameters.F.*log((1-xnS)./xnS) + Ven0 + Ven1 + Ven2 + Ven3 + Ven4 + Ven5 + Ven6 + Ven7 + Ven8 + Ven9 + Ven10 + Ven11 + Ven12;
    V = Vep - Ven - Vo - Vsn - Vsp;
    Vm = V;

    %states
    states=zeros(8,1);
    states(1)=Tb;
    states(2)=Vo;
    states(3)=Vsn;
    states(4)=Vsp;
    states(5)=qnB;
    states(6)=qnS;
    states(7)=qpB;
    states(8)=qpS;
    
    block.OutputPort(1).Data = Vm; % voltage 
    block.OutputPort(2).Data = qnS./(0.6*parameters.('qSMax')); % SOC
    block.OutputPort(3).Data = Tbm; % Temperature (Celsius)  

    % output states
    for i=1:8
        block.OutputPort(4).Data(i) =states(i);
    end
  
%endfunction

function Update(block)
    %get arguments
    parameters = block.DialogPrm(1).Data;
    
    % Extract states
    Tb = block.Dwork(1).Data(1);
    Vo = block.Dwork(1).Data(2);
    Vsn = block.Dwork(1).Data(3);
    Vsp = block.Dwork(1).Data(4);
    qnB = block.Dwork(1).Data(5);
    qnS = block.Dwork(1).Data(6);
    qpB = block.Dwork(1).Data(7);
    qpS = block.Dwork(1).Data(8);
    
    % Constraints
    Tbdot = 0;
    CnBulk = qnB./parameters.VolB;
    CnSurface = qnS./parameters.VolS;
    CpSurface = qpS./parameters.VolS;
    xnS = qnS./parameters.qSMax;
    Ven5 = parameters.An5.*((2.*xnS-1).^(5+1) - (2.*xnS.*5.*(1-xnS))./(2.*xnS-1).^(1-5))./parameters.F;
    xpS = qpS./parameters.qSMax;
    Vep3 = parameters.Ap3.*((2.*xpS-1).^(3+1) - (2.*xpS.*3.*(1-xpS))./(2.*xpS-1).^(1-3))./parameters.F;
    Vep12 = parameters.Ap12.*((2.*xpS-1).^(12+1) - (2.*xpS.*12.*(1-xpS))./(2.*xpS-1).^(1-12))./parameters.F;
    Vep4 = parameters.Ap4.*((2.*xpS-1).^(4+1) - (2.*xpS.*4.*(1-xpS))./(2.*xpS-1).^(1-4))./parameters.F;
    Vep11 = parameters.Ap11.*((2.*xpS-1).^(11+1) - (2.*xpS.*11.*(1-xpS))./(2.*xpS-1).^(1-11))./parameters.F;
    Vep2 = parameters.Ap2.*((2.*xpS-1).^(2+1) - (2.*xpS.*2.*(1-xpS))./(2.*xpS-1).^(1-2))./parameters.F;
    Vep7 = parameters.Ap7.*((2.*xpS-1).^(7+1) - (2.*xpS.*7.*(1-xpS))./(2.*xpS-1).^(1-7))./parameters.F;
    CpBulk = qpB./parameters.VolB;
    Vep8 = parameters.Ap8.*((2.*xpS-1).^(8+1) - (2.*xpS.*8.*(1-xpS))./(2.*xpS-1).^(1-8))./parameters.F;
    qdotDiffusionBSn = (CnBulk-CnSurface)./parameters.tDiffusion;
    qnBdot = - qdotDiffusionBSn;
    Jn0 = parameters.kn.*(1-xnS).^parameters.alpha.*(xnS).^parameters.alpha;
    Ven3 = parameters.An3.*((2.*xnS-1).^(3+1) - (2.*xnS.*3.*(1-xnS))./(2.*xnS-1).^(1-3))./parameters.F;
    qdotDiffusionBSp = (CpBulk-CpSurface)./parameters.tDiffusion;
    Ven0 = parameters.An0.*((2.*xnS-1).^(0+1))./parameters.F;
    Jp0 = parameters.kp.*(1-xpS).^parameters.alpha.*(xpS).^parameters.alpha;
    Ven10 = parameters.An10.*((2.*xnS-1).^(10+1) - (2.*xnS.*10.*(1-xnS))./(2.*xnS-1).^(1-10))./parameters.F;
    Ven7 = parameters.An7.*((2.*xnS-1).^(7+1) - (2.*xnS.*7.*(1-xnS))./(2.*xnS-1).^(1-7))./parameters.F;
    Ven2 = parameters.An2.*((2.*xnS-1).^(2+1) - (2.*xnS.*2.*(1-xnS))./(2.*xnS-1).^(1-2))./parameters.F;
    Ven11 = parameters.An11.*((2.*xnS-1).^(11+1) - (2.*xnS.*11.*(1-xnS))./(2.*xnS-1).^(1-11))./parameters.F;
    Ven8 = parameters.An8.*((2.*xnS-1).^(8+1) - (2.*xnS.*8.*(1-xnS))./(2.*xnS-1).^(1-8))./parameters.F;
    Ven12 = parameters.An12.*((2.*xnS-1).^(12+1) - (2.*xnS.*12.*(1-xnS))./(2.*xnS-1).^(1-12))./parameters.F;
    Ven1 = parameters.An1.*((2.*xnS-1).^(1+1) - (2.*xnS.*1.*(1-xnS))./(2.*xnS-1).^(1-1))./parameters.F;
    Ven4 = parameters.An4.*((2.*xnS-1).^(4+1) - (2.*xnS.*4.*(1-xnS))./(2.*xnS-1).^(1-4))./parameters.F;
    Ven6 = parameters.An6.*((2.*xnS-1).^(6+1) - (2.*xnS.*6.*(1-xnS))./(2.*xnS-1).^(1-6))./parameters.F;
    Ven9 = parameters.An9.*((2.*xnS-1).^(9+1) - (2.*xnS.*9.*(1-xnS))./(2.*xnS-1).^(1-9))./parameters.F;
    Vep0 = parameters.Ap0.*((2.*xpS-1).^(0+1))./parameters.F;
    Vep5 = parameters.Ap5.*((2.*xpS-1).^(5+1) - (2.*xpS.*5.*(1-xpS))./(2.*xpS-1).^(1-5))./parameters.F;
    Vep6 = parameters.Ap6.*((2.*xpS-1).^(6+1) - (2.*xpS.*6.*(1-xpS))./(2.*xpS-1).^(1-6))./parameters.F;
    Vep1 = parameters.Ap1.*((2.*xpS-1).^(1+1) - (2.*xpS.*1.*(1-xpS))./(2.*xpS-1).^(1-1))./parameters.F;
    Vep10 = parameters.Ap10.*((2.*xpS-1).^(10+1) - (2.*xpS.*10.*(1-xpS))./(2.*xpS-1).^(1-10))./parameters.F;
    Vep9 = parameters.Ap9.*((2.*xpS-1).^(9+1) - (2.*xpS.*9.*(1-xpS))./(2.*xpS-1).^(1-9))./parameters.F;
    qpBdot = - qdotDiffusionBSp;
    Ven = parameters.U0n + parameters.R.*Tb./parameters.F.*log((1-xnS)./xnS) + Ven0 + Ven1 + Ven2 + Ven3 + Ven4 + Ven5 + Ven6 + Ven7 + Ven8 + Ven9 + Ven10 + Ven11 + Ven12;
    Vep = parameters.U0p + parameters.R.*Tb./parameters.F.*log((1-xpS)./xpS) + Vep0 + Vep1 + Vep2 + Vep3 + Vep4 + Vep5 + Vep6 + Vep7 + Vep8 + Vep9 + Vep10 + Vep11 + Vep12;
    V = Vep - Ven - Vo - Vsn - Vsp;
    
    %input current -> in the original work is load
    i = block.InputPort(1).Data;
    
    %remaining equations
    qpSdot = i + qdotDiffusionBSp;
    Jn = i./parameters.Sn;
    VoNominal = i.*parameters.Ro;
    Jp = i./parameters.Sp;
    qnSdot = qdotDiffusionBSn - i;
    VsnNominal = parameters.R.*Tb./parameters.F./parameters.alpha.*asinh(Jn./(2.*Jn0));
    Vodot = (VoNominal-Vo)./parameters.to;
    VspNominal = parameters.R.*Tb./parameters.F./parameters.alpha.*asinh(Jp./(2.*Jp0));
    Vsndot = (VsnNominal-Vsn)./parameters.tsn;
    Vspdot = (VspNominal-Vsp)./parameters.tsp;

    % Update state    
    dt=parameters.sampleTime;
    block.Dwork(1).Data(1) = Tb + Tbdot*dt;
    block.Dwork(1).Data(2) = Vo + Vodot*dt;
    block.Dwork(1).Data(3) = Vsn + Vsndot*dt;
    block.Dwork(1).Data(4) = Vsp + Vspdot*dt;
    block.Dwork(1).Data(5) = qnB + qnBdot*dt;
    block.Dwork(1).Data(6) = qnS + qnSdot*dt;
    block.Dwork(1).Data(7) = qpB + qpBdot*dt;
    block.Dwork(1).Data(8) = qpS + qpSdot*dt;

    % Add process noise later on
    % XNew = XNew + dt*N;

%endfunction

