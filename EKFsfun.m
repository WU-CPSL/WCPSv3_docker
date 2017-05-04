function [sys,x0,str,ts,simStateCompliance] = EKFsfun(t,x,u,flag,A,B,C,T,delta_t)

switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes();

  case 1,
    sys=mdlDerivatives();

  case 2,
    sys=mdlUpdate(t,x,u,A,B,C,T,delta_t);

  case 3,
    sys=mdlOutputs(t,x,u);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes()


sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 3;   % x1, x2, count
sizes.NumOutputs     = 2;   % x1, x2
sizes.NumInputs      = 3;   % y,trace, u
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
x0=[0;0;1];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives()

sys = [];

function sys=mdlUpdate(t,x,u,A,B,C,T,delta_t)

Q=0.5*eye(2,2);
R=1;

global jX bX bP jP Kk
jX=[x(1);x(2)];
x_M=u(1);%y measurement
TraceS=u(2);
U=u(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MA=T^(-1)*A*T;
MB=T^(-1)*B;
MC=C*T;
SSMA=delta_t*MA+eye(2);
SSMB=delta_t*MB;

bX=SSMA*jX+SSMB*U;
AJ=SSMA;
bP=AJ*jP*AJ'+Q;
Kk=bP*MC'*(inv(MC*bP*MC'+R));
jX=bX+Kk*(x_M-MC*bX)*TraceS;  
jP=bP-Kk*MC*bP;

x(1)=jX(1);
x(2)=jX(2);
x(3)=x(3)+1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys = x;


function sys=mdlOutputs(t,x,u)

sys = [x(1);x(2)]; %x1,x2

function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)

sys = [];
% end mdlTerminate
