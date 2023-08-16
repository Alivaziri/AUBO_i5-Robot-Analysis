clc;clear;close all

%%  DENAVIT-HARTENBERG PARAMETERS

%    length      twist     angle   offset       range
% ------------------------------------------------------
% 1    0           0         0     theta1        175
% 2    90          0         l1    theta2+90     175
% 3    0           l2        0     theta3        175 
% 4    0           l3        0     theta4-90     175
% 5   -90          0         l4    theta5        175
% 6    90          0         0     theta6        175

%%  FORWARD KINEMATICS

syms a alpha theta d 


T=[cos(theta) -sin(theta) 0 a
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d
    sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d
    0 0 0 1];

syms  theta1 theta2 theta3 theta4 theta5 theta6 l1 l2 l3 l4 l5 l6
T10=subs(T,[alpha,a,d,theta],[0,0,0,theta1]);
T21=subs(T,[alpha,a,d,theta],[pi/2,0,l1,pi/2 + theta2]);
T32=subs(T,[alpha,a,d,theta],[0,l2,0,theta3]);
T43=subs(T,[alpha,a,d,theta],[0,l3,0,theta4-pi/2]);
T54=subs(T,[alpha,a,d,theta],[-pi/2,0,l4,theta5]);
T65=subs(T,[alpha,a,d,theta],[pi/2,0,0,theta6]);

T60=simplify(T10*T21*T32*T43*T54*T65);
Pos = matlabFunction(T60(1:3,4));
orien = matlabFunction(T60(1:3,1:3));

%% ROBOT'S WORKSPACE

t1 = -175*pi/180:0.1:175*pi/180;
t2 = -175*pi/180:0.1:175*pi/180;
t3 = -175*pi/180:0.1:175*pi/180;
t4 = -175*pi/180:0.1:175*pi/180;

n = length(t1)*length(t2)*length(t3)*length(t4);
P = zeros(3,n);

l1=140.5;
l2=408;
l3=376;
l4=102.5;

N=1;
for i=1:length(t1)
    for j=1:length(t2)
        for k=1:length(t3)
            for m=1:length(t4)
                P(:,N) = Pos(l1,l2,l3,l4,t1(i),t2(j),t3(k),t4(m));
                N = N+1;
            end
        end
    end
end
x = P(1,:);
y= P(2,:);
z = P(3,:);
plot3(x,y,z)
xlabel('x'); ylabel('y'); zlabel('z');

figure(2)
subplot(3,1,1);
plot(P(1,:),P(2,:))
xlabel('x'); ylabel('y');

subplot(3,1,2);
plot(P(1,:),P(3,:))
xlabel('x'); ylabel('z');

subplot(3,1,3);
plot(P(2,:),P(3,:))
xlabel('y'); ylabel('z');