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

%% INVERSE KINEMATICS OF ROBOT

 syms px py pz r11 r12 r13 r21 r22 r23 r31 r32 r33 theta1 theta2 theta3 theta4 theta5 theta6 l1 l2 l3 l4 l5 l6
 
 T60_desired=[r11 r12 r13 px
              r21 r22 r23 py
              r31 r32 r33 pz
              0 0 0 1];
          
L1=140.5;   L2=408;  % link length
L3=376;     L4=102.5;

R11=-(7*3^(1/2))/16;   R12=  -1/16;   R13=(3*3^(1/2))/8;  Px=L3/4 + (3^(1/2)*L1)/2 - (3^(1/2)*L4)/40;
R21=-3/16;   R22=(9*3^(1/2))/16;   R23=-1/8;  Py=L1/2 + (3*L4)/4 - (3^(1/2)*L3)/4;
R31=-5/8;   R32= -3^(1/2)/8;   R33= -3/4;  Pz= L2 - L4/2 - (3^(1/2)*L3)/20;


theta=zeros(1,6);   %joint angles

% to check if the desired point is in the workspace or not
if min(x)<Px && Px<max(x) && min(y)<Py && Py<max(y)&& min(z)<Pz && Pz<max(z)
    
    %theta1
    THETA11=subs(atan2(px,py)-atan2(sqrt(px^2+py^2-l1^2),-l1),[px,py,l1],[Px,Py,L1]);
    THETA12=subs(atan2(px,py)-atan2(-sqrt(px^2+py^2-l1^2),-l1),[px,py,l1],[Px,Py,L1]);
%     theta11=mod(THETA11,2*pi);
%     theta12=mod(THETA12,2*pi);
    if THETA11>=deg2rad(-175) && THETA11<=deg2rad(175)
        theta(1)=THETA11;
    elseif THETA12>=deg2rad(-175) && THETA12<=deg2rad(175)
        theta(1)=THETA12;
    else
        disp('out of THETA1 range')
    end
    
    %theta5
    THETA51=subs(atan2(sqrt(1-(r13*cos(theta(1))-r23*cos(theta(1)))^2),r13*sin(theta(1))-r23*cos(theta(1))),[r13,r23],[R13,R23]);
    THETA52=subs(atan2(-sqrt(1-(r13*cos(theta(1))-r23*cos(theta(1)))^2),r13*sin(theta(1))-r23*cos(theta(1))),[r13,r23],[R13,R23]);
    if THETA51>=deg2rad(-175) && THETA51<=deg2rad(175)
        theta(5)=THETA51;
    elseif THETA52>=deg2rad(-175) && THETA52<=deg2rad(175)
        theta(5)=THETA52;
    else
        disp('out of THETA5 range')
    end
    
    %theta6
        THETA6=subs(atan((r12*sin(theta(1))-r22*cos(theta(1)))/(r21*cos(theta(1))-r11*sin(theta(1)))),[r12,r22,r21,r11],[R12,R22,R21,R11]);
        if THETA6>=deg2rad(-175) && THETA6<=deg2rad(175)
            theta(6)=THETA6;
        else
            disp('out of THETA3 range')
        end
 
    %theta(23-4)
    
   THETA23_4=subs(atan2(r12*cos(theta(1))*cos(theta(6))+r11*cos(theta(1))*sin(theta(6)) ...
      +r22*cos(theta(6))*sin(theta(1))+r21*sin(theta(1))*sin(theta(6)),r32*cos(theta(6))+r31*sin(theta(6))) ...
       ,[r12,r11,r22,r21,r32,r31],[R12,R11,R22,R21,R32,R31]);
    
   %theta3
   COSINE=((px*cos(theta(1))+py*sin(theta(1))-l4*sin(THETA23_4))^2+(pz-cos(THETA23_4)*l4)^2-l2^2-l3^2)/2*l2*l3;
   THETA31=subs(atan2(sqrt(1-COSINE^2),COSINE),[px,py,l4,pz,l2,l3],[Px,Py,L4,Pz,L2,L3]);
   THETA32=subs(atan2(-sqrt(1-COSINE^2),COSINE),[px,py,l4,pz,l2,l3],[Px,Py,L4,Pz,L2,L3]);
   if THETA31>=deg2rad(-175) && THETA31<=deg2rad(175)
        theta(3)=THETA31;
    elseif THETA32>=deg2rad(-175) && THETA32<=deg2rad(175)
        theta(3)=THETA32;
    else
        disp('out of THETA3 range')
   end
    
   %theta2
   
    SINE=-l4*cos(THETA23_4)/l3*sin(theta(3))*(1+(l3*cos(theta(3))+l2)/l3*sin(theta(3)));
    THETA21=subs(atan2(SINE,sqrt(1-SINE^2)),[l4,l3,l2],[L4,L3,L2]);
    THETA22=subs(atan2(SINE,-sqrt(1-SINE^2)),[l4,l3,l2],[L4,L3,L2]);
     if THETA21>=deg2rad(-175) && THETA21<=deg2rad(175)
        theta(2)=THETA21;
    elseif THETA22>=deg2rad(-175) && THETA22<=deg2rad(175)
        theta(2)=THETA22;
    else
        disp('out of THETA2 range')
     end
    
   %theta4
    THETA4=theta(2)+theta(3)-THETA23_4;
    if THETA4>=deg2rad(-175) && THETA4<=deg2rad(175)
        theta(4)=THETA4;
         else
        disp('out of THETA4 range')
    end
    
else
    disp('out of workspace')
end
        
disp(theta)