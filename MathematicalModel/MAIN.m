%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a framwork to test control algorithms and mathematical models   %
%                         for Dextair UAV                                 %
%                               By                                        %
%              Ghadeer SHAABAN         Borhan HLEISS                      %
%                          Supervised by:                                 %
%                          Prof. Ahmad HABLY                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General notes %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% You can try and  see results without oppening slx file, just run this   %
% file and see results, you can change and parameter defined here         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Your choices  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%% controller: PID or SMC %%%%%%%%%%%%%%%%%%%%%%%%%   

% If you want to use PID put:  SMCONTROLTRUE=0;
% and to use SMC, put:         SMCONTROLTRUE=1;
% important note, if you want to use SMC, you should disable zero crossing
% dectection (when you run the code, matlab will suggest that to you, Don't
% worry!! 
SMCONTROLTRUE=0;
%SMCONTROLTRUE=1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% trajectory %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

% choose your reference trajectory ( circle, rectagle, heart, flower )

%DesiredReferenceTrajectory='circle';
%DesiredReferenceTrajectory='rectangle';
%DesiredReferenceTrajectory='heart';
DesiredReferenceTrajectory='flower';




%%%%%%%%%%%%%%%%%%%%%%%%% perturpation on spring %%%%%%%%%%%%%%%%%%%%%%%%%%

% If you want perturpation on spring:  perturpationOnSpring=1;
% If not, put:                         perturpationOnSpring=0;

perturpationOnSpring=0;
%perturpationOnSpring=1;


%%%%%%%%%%%%%%%%%%%%%%%%% saturation on actuators %%%%%%%%%%%%%%%%%%%%%%%%%

% If you want saturation on actuators:  saturationOnActuators=1;
% If not, put:                          saturationOnActuators=0;

%saturationOnActuators=0;
saturationOnActuators=1;


%%%%%%%%%%%%%%%%%%%%%%%%% delay on actuators %%%%%%%%%%%%%%%%%%%%%%%%%

% If you want delay on actuators:       delayOnActuators=1;
% If not, put:                          delayOnActuators=0;

delayOnActuators=0;
%delayOnActuators=1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        Dextair parameter                                % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m=2.7;                      % dextair mass (kg)
I=0.0504;                   % moment of inertia with respect to any axis
Ix=I; Iy=I; Iz=I;
k=22.5;                     % stiffness constant (kg/m)
L=0.16;                     % Propeller axis to CoG distance (m)
delta=0.32;                 % Spring to CoG distance (m)
a=1.7*10^(-6);              % Thrust coefficient (1.7?N s^2/rad^2)

g=9.8;                      % gravitational acceleration (m/s^2)
l_0=10;                     % spring rest length (m)
%note: z in the results is not the real one, you should shift by adding Z_0
Z_0=m*g/k+l_0;              % initial position of dextair (m) 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% Controllers  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        PID controllers                                  % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PID_P=12.71;
PID_I=2.12;
PID_D=19;

PID_P_x=PID_P; PID_I_x=PID_I; PID_D_x=PID_D;
PID_P_y=PID_P; PID_I_y=PID_I; PID_D_y=PID_D;
PID_P_z=PID_P; PID_I_z=PID_I; PID_D_z=PID_D;

PID_P_phi=PID_P;     PID_I_phi=PID_I;    PID_D_phi=PID_D;
PID_P_theta=PID_P;   PID_I_theta=PID_I;  PID_D_theta=PID_D;
PID_P_psi=PID_P;     PID_I_psi=PID_I;    PID_D_psi=PID_D;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        Slinding mode control                            % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gamma=3;
lamda=12;
W=0.866;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%% Constrains and perturbations %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%% perturpation on spring %%%%%%%%%%%%%%%%%%%%%%%%%%
% f_x=Amp*sin(Freq*t)
if perturpationOnSpring==1 
    perturpationAmp=20*pi;
    perturpationFreq=10;
else 
    perturpationAmp=0;
    perturpationFreq=0;
end

%%%%%%%%%%%%%%%%%%%%%%%%% saturation on actuators %%%%%%%%%%%%%%%%%%%%%%%%%
if saturationOnActuators==1
    w_max=25000*2*pi/60; %rad/sec
else 
    w_max=10000000000000;
end


%%%%%%%%%%%%%%%%%%%%%%%%% delay on actuators %%%%%%%%%%%%%%%%%%%%%%%%%
if delayOnActuators==1
   to_motor=0.06;
else 
   to_motor=0;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%% Reference trajectory %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Rectangle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Default 
dt=0.1;
L_X=2;T_x=3;
L_Z=1; T_z=2;

T_simulink=2*(T_x+T_z);

t=0:dt:T_simulink;
t=t';
t_1=0:dt:T_x;
t_2=dt:dt:T_z;
t_3=dt:dt:T_x;
t_4=dt:dt:T_z;

trajectory_x=[t_1*L_X/T_x  L_X*ones(1,length(t_2)) (1-t_3/T_x)*L_X zeros(1,length(t_4))]';
trajectory_z=[zeros(1,length(t_1)) t_2*L_Z/T_z  L_Z*ones(1,length(t_3))   (1-t_4/T_z)*L_Z]';
trajectory_y=[zeros(1,length(t))]';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Circle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(DesiredReferenceTrajectory,'circle')
T_simulink=2*pi;
Amp_z=0.1;
Amp_x=1;
bias_x=1;
frequency=1;
dt=0.1;
t=0:dt:T_simulink;
t=t';
trajectory_x=bias_x-Amp_x*sin(frequency*t+pi/2);
trajectory_z=Amp_z*sin(frequency*t);
trajectory_y=[zeros(1,length(t))]';
end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% flower %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(DesiredReferenceTrajectory,'flower')
load('flower_points.mat')
trajectory_x=flower_points(:,1);
n=length(trajectory_x);
trajectory_y=zeros(n,1);
trajectory_z=flower_points(:,2);

T_simulink=n/2;
dt=0.5;
t=0:dt:T_simulink-dt;
t=t';
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Heart %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(DesiredReferenceTrajectory,'heart')
T_simulink=10;
dt=0.1;
t=0:dt:T_simulink;
t=t';
trajectory_x=16*(sin(t)).^3;
trajectory_z=+13*sin(t+pi/2)-5*sin(2*t+pi/2)-2*sin(3*t+pi/2)-sin(4*t+pi/2);
trajectory_y=[zeros(1,length(t))]';

% here we remove saturation as heart dimension higher that the limitation
% when using saturations
w_max=10000000000000;
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% initial points %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x0=trajectory_x(1);
y0=trajectory_y(1);
z0=trajectory_z(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%% apply on simulink and reslutls %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sim("simple_dextair.slx", T_simulink) ;


figure() ;
sgtitle("Trajectory tracking, X4 nonlinear model") 
subplot 121 ;
plot3(simout_x_ref, simout_y_ref, simout_z_ref) ; grid on ; 
hold on ;
plot3(simout_x, simout_y, simout_z) ;
legend("Reference trajectory", "Simulated X4 trajectory") ;
subplot 222 ;
plot(simout_x_ref, simout_y_ref) ;title("Plan (x,y)") ; grid on ; 
hold on ;
plot(simout_x, simout_y) ;
legend("Reference trajectory", "Simulated X4 trajectory") ;
subplot 224 ;
plot(simout_x_ref, simout_z_ref) ; grid on ; title("Plan (x,z)") ;
hold on ;
plot(simout_x, simout_z) ;
legend("Reference trajectory", "Simulated  trajectory") ;

% separated plots
figure() ;
plot3(simout_x_ref, simout_y_ref, simout_z_ref) ; grid on ; 
hold on ;
plot3(simout_x, simout_y, simout_z) ;
legend("Reference trajectory", "Simulated trajectory") ;
title("Trajectory tracking")
figure();
plot(simout_x_ref, simout_y_ref) ;title("Plan (x,y)") ; grid on ; 
hold on ;
plot(simout_x, simout_y) ;
legend("Reference trajectory", "Simulated  trajectory") ;

figure() ;
plot(simout_x_ref, simout_z_ref) ; grid on ; title("Plan (x,z)") ;
hold on ;
plot(simout_x, simout_z) ;
legend("Reference trajectory", "Simulated  trajectory") ;



%% actuators 
dt=T_simulink/length(simout_x_ref);
t=0:dt:T_simulink-dt;
figure() ;   title("rotors rotational speeds") ;
plot(t, www(:,1)) ; grid on ;
hold on ;
plot(t, www(:,2)) ;
plot(t, www(:,3)) ;
plot(t, www(:,4)) ;
plot(t, www(:,5)) ;
plot(t, www(:,6)) ;
xlabel('time (sec)');
ylabel('rotational speeds (rad.s^{-1})');
legend("\omega_1", "\omega_2","\omega_3","\omega_4","\omega_5","\omega_6") ;

