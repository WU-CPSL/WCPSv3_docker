function [sys,x0,str,ts,simStateCompliance] = MPCControllersfun(t,x,u,flag,A,B,C,T,uini,Xsp,delta_t)

switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(uini);

  case 1,
    sys=mdlDerivatives(t,x,u);

  case 2,
    sys=mdlUpdate(t,x,u,A,B,C,T,Xsp,delta_t);

  case 3,
    sys=mdlOutputs(t,x,u);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(uini)


sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 3;   %control input, stepcounter, control input
sizes.NumOutputs     = 5;   %control inputs buffer
sizes.NumInputs      = 2;   %x1, x2
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);


x0  = [uini;1;uini];
global Buff Last
 for k=1:size(Buff,2);
     Buff(k)=x0(1);
 end
Last=x0(1);
 
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u)

sys = [];

function sys=mdlUpdate(t,x,u,A,B,C,T,Xsp,delta_t)
global Buff Last
x1=u(1);
x2=u(2);

Nstates = 2;
Horizon = 6;
Ninputs = 1;
Oidxs=[1,2];
Nopt = Nstates * Horizon + Ninputs * ( Horizon - 1 );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

MA=T^(-1)*A*T;
MB=T^(-1)*B;
SSMA=delta_t*MA+eye(2);
SSMB=delta_t*MB;
SSMC=[1 0;0 1];
ref=Xsp;

H = zeros( Nopt, Nopt );
f = zeros( Nopt, 1 );
MQ =0.01* eye( Nstates );
MQ(2,2) = 1;
MS = 0.1 * eye( Nstates );
MS(2,2) =10;
MR = 0.2 * eye( Ninputs );


for k = 1:Horizon-1,
  H( (k-1)*Nstates + Oidxs, ...
     (k-1)*Nstates + Oidxs ) = 2 * SSMC * MQ * SSMC';
  H( Nstates*Horizon + (k-1)*Ninputs + (1:Ninputs), ...
     Nstates*Horizon + (k-1)*Ninputs + (1:Ninputs) ) = 2 * MR;
  f( (k-1)*Nstates + (1:Nstates) ) = - 2 * ref' * SSMC * MQ;
end
H( (Horizon-1)*Nstates + Oidxs, ...
   (Horizon-1)*Nstates + Oidxs ) = 2 * SSMC * MS * SSMC';
f( (Horizon-1)*Nstates + (1:Nstates) ) = - 2 * ref' * SSMC * MS;

Aeq = zeros( Nstates * Horizon, Nopt );
beq = zeros( Nstates * Horizon, 1 );

for k = 1:(Horizon-1),
  Aeq( (k-1)*Nstates + (1:Nstates), ...
       (k-1)*Nstates + (1:Nstates) ) = -SSMA;
  Aeq( (k-1)*Nstates + (1:Nstates), ...
           k*Nstates + (1:Nstates) ) = eye(Nstates);
  Aeq( (k-1)*Nstates + (1:Nstates), ...
       Nstates*Horizon + (k-1)*Ninputs + (1:Ninputs) ) = -SSMB;
end
Aeq( (Horizon-1)*Nstates + (1:Nstates), ...
                            1:Nstates ) = eye(Nstates);
beq( (Horizon-1)*Nstates + (1:Nstates) ) = [ x1,x2 ];

lb = zeros( Nopt, 1 );
ub = zeros( Nopt, 1 );
for k = 1:Horizon,
    lb( (k-1)*Nstates + (1:Nstates) ) = [-inf; -inf];
    ub( (k-1)*Nstates + (1:Nstates) ) = [ inf; inf];% upper and lower bound for States and Z
end

for k = 1:Horizon-1,
    ub( Nstates*Horizon + (k-1)*Ninputs + (1:Ninputs) ) = 10;
    lb( Nstates*Horizon + (k-1)*Ninputs + (1:Ninputs) ) = -10;
end

z=zeros(Nopt,1);
clear model;
clear params;
model.Q = sparse(0.5*H);
model.A = sparse(Aeq);
model.obj = f';
model.rhs = beq';
model.lb=lb; 
model.ub=ub;
model.sense = '=';
params.outputflag = 0;
params.DualReductions=0;

gurobi_write(model, 'qp.lp');

results = gurobi(model,params);

for v=1:Nopt
     z(v)=results.x(v);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

for k=1:size(Buff,2)
     Buff(k)=z(Nstates * Horizon + Ninputs * k);
end

                              
 x(1) = Buff(1);
Last=Buff(size(Buff,2));
 x(2) = x(2) + 1;
 x(3) = z(Nstates*Horizon + 1);
sys = [x];

%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
global Buff;
sys = [Buff];



function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;
sys = t + sampleTime;



function sys=mdlTerminate(t,x,u)

sys = [];
% end mdlTerminate
