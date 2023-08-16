clc; clear; close all
%% LAGRANGE 

%% parameters

syms m r L m1 m2 m3 m4 m5 m6 r1 r2 r3 r4 r5 r6 L1 L2 L3 L4 L5 g

m1=5.51; m2=7.57; m3=3.86; m4=3.95; m5=2.39; m6=2.39;
r1=0.049; r2=0.049; r3=0.435; r4=0.375; r5=0.375; r6=0.375;
L1=1.025;L2=0.408; L3=0.376; L4=1.025; L5=1.025;

% I_formula=(m*r^2)/4+(m*L^2)/12;
% I1=subs(I_formula,[m,r,L],[m1,r1,0]);
% I2=subs(I_formula,[m,r,L],[m2,r2,L2]);
% I3=subs(I_formula,[m,r,L],[m3,r3,0]);
% I4=subs(I_formula,[m,r,L],[m4,r4,L4]);
% I5=subs(I_formula,[m,r,L],[m5,r5,L5]);
% I6=subs(I_formula,[m,r,L],[m6,r6,0]);

m=[m1 m2 m3 m4 m5 m6];
I(:,:,1)=[1322951/400000000,0,0;
 0,1322951/400000000,0
 0,0,1322951/400000000];
I(:,:,2)=[43821973/400000000,0,0
 0,43821973/400000000,0
 0,0,43821973/400000000];
I(:,:,3)=[1460817/800000000,0,0
 0,1460817/800000000,0
 0,0,1460817/800000000];
I(:,:,4)=[46531/9600000,0,0
 0,46531/9600000,0
 0,0,46531/9600000];
I(:,:,5)=[140771/48000000,0,0
 0,140771/48000000,0
 0,0,140771/48000000];
I(:,:,6)=[2151/2560000,0,0
 0,2151/2560000,0
 0,0,2151/2560000];

%%  FORWARD KINEMATICS

syms a alpha theta d 


T=[cos(theta) -sin(theta) 0 a
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d
    sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d
    0 0 0 1];

l1=140.5;
l2=408;
l3=376;
l4=102.5;

syms  t1 t2 t3 t4 t5 t6 L1 L2 L3 L4
% t1=theta1; t2=theta2; t3=theta3; t4=theta4; t5=theta5; t6=theta6;
T10=subs(T,[alpha,a,d,theta],[0,0,0,t1]);
T21=subs(T,[alpha,a,d,theta],[pi/2,0,l1,pi/2 + t2]);
T32=subs(T,[alpha,a,d,theta],[0,l2,0,t3]);
T43=subs(T,[alpha,a,d,theta],[0,l3,0,-pi/2+t4]);
T54=subs(T,[alpha,a,d,theta],[-pi/2,0,l4,t5]);
T65=subs(T,[alpha,a,d,theta],[pi/2,0,0,t6]);
T76=subs(T,[alpha,a,d,theta],[0,0,94,0]);

T20=simplify(T10*T21);
T30=simplify(T10*T21*T32);
T40=simplify(T10*T21*T32*T43);
T50=simplify(T10*T21*T32*T43*T54);
T60=simplify(T10*T21*T32*T43*T54*T65);
T70=simplify(T10*T21*T32*T43*T54*T65*T76);

pc=[0 L2/2 0 0 0 0
    0 0 0 0 -L3/2 0
    0 0 -L1 -L1 -L4 0
    1 1 1 1 1 1];

T1C1=[eye(4,3),pc(1:4,1)];
T2C2=[eye(4,3),pc(1:4,2)];
T3C3=[eye(4,3),pc(1:4,3)];
T4C4=[eye(4,3),pc(1:4,4)];
T5C5=[eye(4,3),pc(1:4,5)];
T6C6=[eye(4,3),pc(1:4,6)];

T0C1=simplify(T10*T1C1);
T0C2=simplify(T10*T20*T2C2);
T0C3=simplify(T10*T21*T32*T3C3);
T0C4=simplify(T10*T21*T32*T43*T4C4);
T0C5=simplify(T10*T21*T32*T43*T54*T5C5);
T0C6=simplify(T10*T21*T32*T43*T54*T65*T6C6);

%% ROTATION MATRIX

R10=simplify(T10(1:3,1:3));
R21=simplify(T21(1:3,1:3));
R32=simplify(T32(1:3,1:3));
R43=simplify(T43(1:3,1:3));
R54=simplify(T54(1:3,1:3));
R65=simplify(T65(1:3,1:3));

R=[R10 R21 R32 R43 R54 R65];
Z=[0,0,1]';


%% JACOBIAN

z(1:3,1)=T10(1:3,3);
z(1:3,2)=T20(1:3,3);
z(1:3,3)=T30(1:3,3);
z(1:3,4)=T40(1:3,3);
z(1:3,5)=T50(1:3,3);
z(1:3,6)=T60(1:3,3);

o(1:3,1)=T10(1:3,4);
o(1:3,2)=T20(1:3,4);
o(1:3,3)=T30(1:3,4);
o(1:3,4)=T40(1:3,4);
o(1:3,5)=T50(1:3,4);
o(1:3,6)=T60(1:3,4);
% oe=T70(1:3,4);

oc(1:3,1)=T0C1(1:3,4);
oc(1:3,2)=T0C2(1:3,4);
oc(1:3,3)=T0C3(1:3,4);
oc(1:3,4)=T0C4(1:3,4);
oc(1:3,5)=T0C5(1:3,4);
oc(1:3,6)=T0C6(1:3,4);

jv(:,:,1)= [cross(z(1:3,1),oc(1:3,1)-o(1:3,1)) , [0;0;0] ,[0;0;0] , [0;0;0] ,[0;0;0],[0;0;0]];
jv(:,:,2)= [cross(z(1:3,1),oc(1:3,2)-o(1:3,1)) , cross(z(1:3,2),oc(1:3,2)-o(1:3,2)) ,[0;0;0],[0;0;0],[0;0;0],[0;0;0] ];
jv(:,:,3)= [cross(z(1:3,1),oc(1:3,3)-o(1:3,1)) , cross(z(1:3,2),oc(1:3,3)-o(1:3,2)) , cross(z(1:3,3),oc(1:3,3)-o(1:3,3)) , [0;0;0],[0;0;0],[0;0;0] ];
jv(:,:,4)= [cross(z(1:3,1),oc(1:3,4)-o(1:3,1)) , cross(z(1:3,2),oc(1:3,4)-o(1:3,2)) , cross(z(1:3,3),oc(1:3,4)-o(1:3,3)) , cross(z(1:3,4),oc(1:3,4)-o(1:3,4)),[0;0;0],[0;0;0] ];
jv(:,:,5)= [cross(z(1:3,1),oc(1:3,5)-o(1:3,1)) , cross(z(1:3,2),oc(1:3,5)-o(1:3,2)) , cross(z(1:3,3),oc(1:3,5)-o(1:3,3)) , cross(z(1:3,4),oc(1:3,5)-o(1:3,4)),cross(z(1:3,5),oc(1:3,5)-o(1:3,5)),[0;0;0] ];
jv(:,:,6)= [cross(z(1:3,1),oc(1:3,6)-o(1:3,1)) , cross(z(1:3,2),oc(1:3,6)-o(1:3,2)) , cross(z(1:3,3),oc(1:3,6)-o(1:3,3)) , cross(z(1:3,4),oc(1:3,6)-o(1:3,4)),cross(z(1:3,5),oc(1:3,6)-o(1:3,5)),cross(z(1:3,6),oc(1:3,6)-o(1:3,6)) ];

jw(:,:,1) = [z(1:3,1), [0;0;0] , [0;0;0],[0;0;0],[0;0;0],[0;0;0]];
jw(:,:,2) = [z(1:3,1), z(1:3,2) ,[0;0;0], [0;0;0],[0;0;0],[0;0;0]];
jw(:,:,3) = [z(1:3,1), z(1:3,2) , z(1:3,3) ,[0;0;0],[0;0;0],[0;0;0]];
jw(:,:,4) = [z(1:3,1), z(1:3,2) , z(1:3,3) ,z(1:3,4),[0;0;0],[0;0;0]];
jw(:,:,5) = [z(1:3,1), z(1:3,2) , z(1:3,3) ,z(1:3,4),z(1:3,5),[0;0;0]];
jw(:,:,6) = [z(1:3,1), z(1:3,2) , z(1:3,3) ,z(1:3,4),z(1:3,5),z(1:3,6)];

J=[jv;jw];
J=simplify(J);

M = sym(zeros(6,6));

for i = 1:6
 M(:,:,i) = m(i)*jv(:,:,i).'*jv(:,:,i) + ...
        jw(:,:,i).'*R(:,i:i+2)*I(:,:,i)*R(:,i:i+2).'*jw(:,:,i);
end

M =simplify(M(:,:,1)+M(:,:,2)+M(:,:,3)+M(:,:,4)+M(:,:,5)+M(:,:,6));

%% velocity matrix

syms td1 td2 td3 td4 td5 td6
theta=[t1 t2 t3 t4 t5 t6];

c = sym(zeros(6,6,6));

for i=1:6
    for j=1:6
        for k=1:6
            c(i,j,k) = 0.5*(diff(M(k,j),theta(i))+diff(M(k,i),theta(j))-...
            diff(M(i,j),theta(k)));
        end
    end
end

thetad=[td1 td2 td3 td4 td5 td6 ];
C = sym(zeros(6,6));

for i=1:6
    for j=1:6
 
        C(i,j)=c(i,j,1)*thetad(1)+c(i,j,2)*thetad(2)+c(i,j,3)*thetad(3)...
            +c(i,j,4)*thetad(4)+c(i,j,5)*thetad(5)+c(i,j,6)*thetad(6);
    end
end

C=vpa(C);
V=C*thetad.';

%% Gravity;
g=[0 0 -9.81];
U=[-m1*g*oc(1:3,1);
   -m2*g*oc(1:3,2);
   -m3*g*oc(1:3,3);
   -m4*g*oc(1:3,4);
   -m5*g*oc(1:3,5);
   -m6*g*oc(1:3,6)];

g_k=sum(U);
G=sym(zeros(1,6));

for i=1:6
 G(i)=diff(g_k,theta(i));
end

G=vpa(G);

G=simplify(G);

%% FORWARD DYNAMICS
% 
% syms t1 t2 t3 t4 t5 t6 ...
%     td1 td2 td3 td4 td5 td6 ...
%     tdd1 tdd2 tdd3 tdd4 tdd5 tdd6
% 
% % td1=thetad1; td2=thetad2; td3=thetad3; td4=thetad4; td5=thetad5; td6=thetad6;
% % tdd1=thetadd1; tdd2=thetadd2; tdd3=thetadd3; tdd4=thetadd4; tdd5=thetadd5; tdd6=thetadd6;
% 
% theta=[t1 t2 t3 t4 t5 t6];
% thetad=[td1 td2 td3 td4 td5 td6 ];
% thetadd=[tdd1 tdd2 tdd3 tdd4 tdd5 tdd6];
% 
% syms t
% theta1=0;
% theta2=t;
% theta3=0;
% theta4=0;
% theta5=0;
% theta6=0;
% 
% trajectory=[theta1 theta2 theta3 theta4 theta5 theta6];
% velocity=diff(trajectory,t);
% acceleration=diff(velocity,t);
% 
% time=1:0.1:10;
% taw=(M*thetadd.')+V+G.';
% torque=subs(taw,[theta,thetad,thetadd,L1,L2,L3,L4],[trajectory,velocity,acceleration,l1,l2,l3,l4]);
% torque_time=subs(torque,t,time);
% 
% for i=1:6
% subplot(3,2,i);
%  plot(time,torque_time(i,:),'LineWidth',1);
%  grid on;
%  xlabel('time (s)');
%  ylabel(['Torque ',num2str(i),' (N.m)']);
% end
% 

