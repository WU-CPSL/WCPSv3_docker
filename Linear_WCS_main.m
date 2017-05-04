clc;
clear all;
close all;
warning off

Tf = 50;              
sensor_sampling_rate=5; %sampling rate
delta_t=1/sensor_sampling_rate; %control period
step_count=Tf*sensor_sampling_rate; %total step count
sen_num = 2; % sensor number 


A=[-3 0;0 0.2];
B=[1; 1];
T=[1 1;0 1];
C=[1 1];

MA=T^(-1)*A*T;
MB=T^(-1)*B;
MC=C*T;


x1ini=2;
x2ini=5;
Xini=[x1ini;x2ini];
uini=0; 

x1sp=5.6;
x2sp=-5.2;

Xsp=[x1sp;x2sp];

%% RUN Wireless  Process Control Simulation
tic


    
global ydelay_global
ydelay_global = zeros(9,1);

global jX bX bP jP bZ Kk Buff Last Buff2
jX=[5;5];
bX=[0;0];
bP=eye(2);
jP=eye(2);
bZ=0;
Kk=[0;0];
Buff=zeros(1,5);
Last=0;
Buff2=zeros(1,5);


Ydelay = zeros(step_count+1,sen_num);
Delay = 0;
ranD=[4 2 2 2 2]; %delayed time steps 

    
    delay1 = zeros(step_count+1,sen_num+1);

    yin=0;
    yin_d=0;
    ystore = Ydelay;
    count = 1;
    i = 0;

    structure.i = 0;
    structure.count = count;
    structure.yin = yin;
    structure.yin_d = yin_d;
    structure.delay1 = delay1;
    structure.ranD = ranD;
    
    option = simset('solver','ode4','FixedStep',delta_t);
    sim('WCS_Simulink.mdl',[0 Tf]);

  
ft_size = 20;
line_width = 2;

set(gca, 'FontSize', ft_size);
plot(TIME,XOUT);
hold on
title('Respond Characteristics of Linear System');
legend('x1','x2');
xlabel('t/s', 'FontSize',ft_size);
ylabel('Physical States', 'FontSize',ft_size);
hold on
plot(0:0.2:Tf, ones(1, Tf/0.2+1)*x1sp, 'b--');
plot(0:0.2:Tf, ones(1, Tf/0.2+1)*x2sp, 'g--');

toc



