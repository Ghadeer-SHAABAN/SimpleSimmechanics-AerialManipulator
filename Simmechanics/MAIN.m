clear all; close all; clc;


%% simple TF
s=tf('s');
Hs=1/s^2;
%% dextair variables:
m=2.07;
I=0.0504;
Ix=I; Iy=I; Iz=I;
k=22.5; 
k=0
L=0.16;
delta=0.32;
a=1.7*10^(-6);
F_max=9;

g=9.8;
l_0=10;
Z_0=m*g/k+l_0;

% body charachtaristic 
xl=0.01; %
V=xl*xl*(2*delta);
density=m/3/V;

I=(2/9*delta*delta)*m
Ix=I; Iy=I; Iz=I;


%% PID simple:
% from PID tuner
PID_P=12.71;
PID_I=2.12;
PID_D=19;

PID_P_x=PID_P; PID_I_x=PID_I; PID_D_x=PID_D;
PID_P_y=PID_P; PID_I_y=PID_I; PID_D_y=PID_D;
PID_P_z=PID_P; PID_I_z=PID_I; PID_D_z=PID_D;

PID_P_phi=PID_P;     PID_I_phi=PID_I;    PID_D_phi=PID_D;
PID_P_theta=PID_P;   PID_I_theta=PID_I;  PID_D_theta=PID_D;
PID_P_psi=PID_P;     PID_I_psi=PID_I;    PID_D_psi=PID_D;