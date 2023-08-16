clc;clear;close all

%%  DENAVIT-HARTENBERG PARAMETERS

%    length      twist     angle   offset
% -----------------------------------------
% 1    0           0         0     theta1
% 2    90          0         l1    theta2+90
% 3    0           l2        0     theta3
% 4    0           l3        0     theta4-90
% 5   -90          0         l4    theta5
% 6    90          0         0     theta6

%%  ROBOT'S WORKSPACE




%%  FORWARD KINEMATICS
syms theta1 theta2 theta3 theta4 theta5 theta6 l1 l2 l3 l4 l5 l6
T10=[cosd(theta1) -sind(theta1) 0 0
     sind(theta1) cosd(theta1) 0 0
     0 0 1 0
     0 0 0 1];
T21=[-sind(theta2) -cosd(theta2) 0 0
     0 0 -1 -l1
     cosd(theta2) -sind(theta2) 0 0
     0 0 0 1];
T32=[cosd(theta3) -sind(theta3) 0 l2
     sind(theta3) cosd(theta3) 0 0
     0 0 1 0
     0 0 0 1];
T43=[sind(theta4) -cosd(theta4) 0 l3
     -cosd(theta4) sind(theta4) 0 0
     0 0 1 0
     0 0 0 1];
T54=[cosd(theta5) -sind(theta5) 0 0
     0 0 1 l4
     -sind(theta5) -cosd(theta5) 0 0
     0 0 0 1];
T65=[cosd(theta6) -sind(theta6) 0 0
     0 0 -1 0
     sind(theta6) cosd(theta6) 0 0
     0 0 0 1];
 T60=T10*T21*T32*T43*T54*T65;
 T60=simplify(T60);
 
%% INPUT REVOLUTE VARIABLES

Theta1=input('theta1 :');
Theta2=input('theta2 :');
Theta3=input('theta3 :');
Theta4=input('theta4 :');
Theta5=input('theta5 :');
Theta6=input('theta6 :');

T60=subs(T60,[theta1,theta2,theta3,theta4,theta5,theta6],[Theta1,Theta2,Theta3,Theta4,Theta5,Theta6]);
disp(T60)
