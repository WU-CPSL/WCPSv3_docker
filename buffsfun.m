function [sys,x0,str,ts,simStateCompliance] = buffsfun(t,x,u,flag)

switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes();

  case 1,
    sys=mdlDerivatives(t,x,u);

  case 2,
    sys=mdlUpdate(t,x,u);

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
sizes.NumDiscStates  = 5;% current pump voltage, stepcounter, ideal pump voltage
sizes.NumOutputs     = 1;   %wired voltage
sizes.NumInputs      = 6;   %x1, x2
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);

x0=zeros(1,5);
 
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u)

sys = [];

function sys=mdlUpdate(t,x,u)
global Buff2 Last
Atrace=u(6);
 
if Atrace==1
      for k=1:size(Buff2,2)
               Buff2(k)=u(k);
      end
  else
      for k=1:(size(Buff2,2)-1)
              Buff2(k)=Buff2(k+1);
      end
      Buff2(size(Buff2,2))=Last;                   
 end

for k=1:5
    x(k)=Buff2(k);
end
sys=[x];

%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = [x(1)];



function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;



function sys=mdlTerminate(t,x,u)

sys = [];
% end mdlTerminate
