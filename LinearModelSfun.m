function [sys,x0,str,ts,simStateCompliance] = LinearModelSfun(t,x,u,flag,A,B,C,T,Xini)


switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(Xini);

  case 1,
    sys=mdlDerivatives(t,x,u,A,B,C,T);

  case 2,
    sys=mdlUpdate(t,x,u);

  case 3,
    sys=mdlOutputs(t,x,u,C,T);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end



function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(Xini)


sizes = simsizes;

sizes.NumContStates  = 2;   %x1, x2
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;   %x1, x2, y
sizes.NumInputs      = 1;   %u
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = Xini;  

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';



function sys=mdlDerivatives(t,x,u,A,B,C,T)

ddd=0;%initial value

ddd=T^(-1)*A*T*x+T^(-1)*B*u;

sys=[ddd];



function sys=mdlUpdate(t,x,u)

sys = [];



function sys=mdlOutputs(t,x,u,C,T)

sys = [x(1);x(2);C*T*x];



function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;



function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
