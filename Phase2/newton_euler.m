clc;clear;close all

%% parameters

syms m r L m1 m2 m3 m4 m5 m6 r1 r2 r3 r4 r5 r6 L1 L2 L3 L4 L5 g

m1=5.51; m2=7.57; m3=3.86; m4=3.95; m5=2.39; m6=2.39;
r1=0.049; r2=0.049; r3=0.0435; r4=0.0375; r5=0.0375; r6=0.0375;
L2=0.408; L3=0.376; L4=0.1025; L5=0.1025;

% I_formula=(m*r^2)/4+(m*L^2)/12;
% I1=subs(I_formula,[m,r,L],[m1,r1,0]);
% I2=subs(I_formula,[m,r,L],[m2,r2,L2]);
% I3=subs(I_formula,[m,r,L],[m3,r3,0]);
% I4=subs(I_formula,[m,r,L],[m4,r4,L4]);
% I5=subs(I_formula,[m,r,L],[m5,r5,L5]);
% I6=subs(I_formula,[m,r,L],[m6,r6,0]);

m=[m1 m2 m3 m4 m5 m6];
% I=[I1 I2 I3 I4 I5 I6];
I(:,:,1)=[1322951/4000000000,0,0;
 0,1322951/4000000000,0
 0,0,1322951/4000000000];
I(:,:,2)=[43821973/4000000000,0,0
 0,43821973/4000000000,0
 0,0,43821973/4000000000];
I(:,:,3)=[1460817/8000000000,0,0
 0,1460817/8000000000,0
 0,0,1460817/8000000000];
I(:,:,4)=[46531/96000000,0,0
 0,46531/96000000,0
 0,0,46531/96000000];
I(:,:,5)=[140771/480000000,0,0
 0,140771/480000000,0
 0,0,140771/480000000];
I(:,:,6)=[2151/25600000,0,0
 0,2151/25600000,0
 0,0,2151/25600000];

%%  FORWARD KINEMATICS

syms a alpha theta d 


T=[cos(theta) -sin(theta) 0 a
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d
    sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d
    0 0 0 1];

l1=0.1405;
l2=0.408;
l3=0.376;
l4=0.1025;

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

%% ROTATION MATRIX

R01=T10(1:3,1:3).';
R12=T21(1:3,1:3).';
R23=T32(1:3,1:3).';
R34=T43(1:3,1:3).';
R45=T54(1:3,1:3).';
R56=T65(1:3,1:3).';
R67=T76(1:3,1:3).';

R=[R01 R12 R23 R34 R45 R56 R67];
% Z=[0,0,1]';

%% INITIAL VALUES

w0=[0 0 0]';
wd0=[0 0 0]';
% g=9.81;
vd0_0=[0,0,g]';  % vdot 0 in base frame
pc=[0 L2/2 0 0 0 0
    0 0 0 0 -L3/2 0
    0 0 -L1 -L1 -L4 0];

% p(1:3,1)=T10(1:3,4);
% p(1:3,2)=T20(1:3,4);
% p(1:3,3)=T30(1:3,4);
% p(1:3,4)=T40(1:3,4);
% p(1:3,5)=T50(1:3,4);
% p(1:3,6)=T60(1:3,4);

p(1:3,1)=T10(1:3,4);
p(1:3,2)=T21(1:3,4);
p(1:3,3)=T32(1:3,4);
p(1:3,4)=T43(1:3,4);
p(1:3,5)=T54(1:3,4);
p(1:3,6)=T65(1:3,4);

z(1:3,1)=T10(1:3,3);
z(1:3,2)=T20(1:3,3);
z(1:3,3)=T30(1:3,3);
z(1:3,4)=T40(1:3,3);
z(1:3,5)=T50(1:3,3);
z(1:3,6)=T60(1:3,3);

syms a
w(3,6)=a;
wd(3,6)=a;
vd(3,6)=a;
vdc(3,6)=a;
F(3,6)=a;
N(3,6)=a;
f(3,7)=a;
n(3,7)=a;
tao(6,1)=a;

f(1:3,7)=0;
n(1:3,7)=0;

%% VELOCITY PROPAGATION
syms t1 t2 t3 t4 t5 t6 ...
    td1 td2 td3 td4 td5 td6 ...
    tdd1 tdd2 tdd3 tdd4 tdd5 tdd6

% td1=thetad1; td2=thetad2; td3=thetad3; td4=thetad4; td5=thetad5; td6=thetad6;
% tdd1=thetadd1; tdd2=thetadd2; tdd3=thetadd3; tdd4=thetadd4; tdd5=thetadd5; tdd6=thetadd6;

theta=[t1 t2 t3 t4 t5 t6];
thetad=[td1 td2 td3 td4 td5 td6 ];
thetadd=[tdd1 tdd2 tdd3 tdd4 tdd5 tdd6];

for i=2:6
    w(1:3,1)=R(:,1:3)*w0+thetad(1)*z(:,1);
    w(1:3,i)=R(:,i:i+2)*w(1:3,i-1)+thetad(i)*z(:,i);
    wd(1:3,1)=R(:,1:3)*wd0+cross(R(:,1:3)*w0,thetad(1)*z(:,1))+thetadd(1)*z(:,1);
    wd(1:3,i)=R(:,i:i+2)*wd(1:3,i-1)+cross(R(:,i:i+2)*w(1:3,i-1 ),thetad(i)*z(:,i))+thetadd(i)*z(:,i);
    vd(1:3,1)=R(:,1:3)*(cross(wd0,p(1:3,1))+cross(w0,cross(w0,p(1:3,1)))+vd0_0);
    vd(1:3,i)=R(:,i:i+2)*(cross(wd(1:3,i-1),p(1:3,i))+cross(w(1:3,i-1),cross(w(1:3,i-1),p(1:3,i)))+vd(1:3,i-1));
    vdc(1:3,1)=cross(wd0,p(1:3,1)+cross(w0,cross(w0,p(1:3,1)))+vd0_0);
    vdc(1:3,i)=cross(wd(1:3,i),pc(1:3,i))+cross(w(1:3,i),cross(w(1:3,i),pc(1:3,i)))+vd(1:3,i);
    F(1:3,i)=m(i)*vdc(1:3,i);
    N(1:3,i)=I(:,:,i)*wd(1:3,i)+cross(w(1:3,i),I(:,:,i)*w(1:3,i));
end


w=simplify(w);
wd=simplify(wd);
vd=simplify(vd);
vdc=simplify(vdc);
F=simplify(F);
N=simplify(N);

for i=6:-1:1
    f(1:3,i)=R(:,i+1:i+3).'*f(1:3,i+1)+F(1:3,i);
    n(1:3,i)=N(1:3,i)+R(:,i+1:i+3).'*n(1:3,i+1)+cross(pc(1:3,i),F(1:3,i))+cross(p(1:3,i),R(:,i+1:i+3).'*f(1:3,i+1));
end


for i=6:-1:1
    tao(i,1)=n(1:3,i).'*z(:,i);
end

% f=simplify(f);
% n=simplify(n);

%% DYNAMICS EQUATION IN JOINT SPACE
M(6,6)=a;
for i=1:6
    M(i,1:6)=jacobian(tao(i,1),thetadd(1,1:6));
end

% M=simplify(M);
% G=simplify(diff(tao,g)*g);
% V=simplify(tao-M*thetadd'-G);


% %% JACOBIAN
% 
% z(1:3,1)=([T10(1,3); T10(2,3) ;T10(3,3)]);
% z(1:3,2)=([T20(1,3); T20(2,3) ;T20(3,3)]);
% z(1:3,3)=[T30(1,3); T30(2,3) ;T30(3,3)];
% z(1:3,4)=[T40(1,3); T40(2,3) ;T40(3,3)];
% z(1:3,5)=[T50(1,3); T50(2,3) ;T50(3,3)];
% z(1:3,6)=[T60(1,3); T60(2,3) ;T60(3,3)];
% 
% o(1:3,1)=[T10(1,4); T10(2,4) ;T10(3,4)];
% o(1:3,2)=[T20(1,4); T20(2,4) ;T20(3,4)];
% o(1:3,3)=[T30(1,4); T30(2,4) ;T30(3,4)];
% o(1:3,4)=[T40(1,4); T40(2,4) ;T40(3,4)];
% o(1:3,5)=[T50(1,4); T50(2,4) ;T50(3,4)];
% o(1:3,6)=[T60(1,4); T60(2,4) ;T60(3,4)];
% oe=[T70(1,4); T70(2,4) ;T70(3,4)];
% 
% syms a
% jv(1:3,1:6)=a;
% jw(1:3,1:6)=a;
% 
% for i=1:6
%     jv(1:3,i)=simplify(cross(z(1:3,i),(oe-o(1:3,i))));
%     jw(1:3,i)=z(1:3,i);
% end
% 
% J=[jv;jw];
% J=simplify(J);
% J_INV=inv(J);
% 
% Jd=diff(J,t1)*td1+diff(J,t2)*td2+diff(J,td3)*td3+diff(J,t4)*td4 ...
%     +diff(J,td5)*td5+diff(J,t6)*td6;
% 
% %% DYNAMIC EQUATION IN CARTESIAN SPACE
% 
% MX=M*J_INV;
% VX=V-MX*Jd*thetadd';
% GX=G;

% %% FORWARD DYNAMICS
% 
% syms t
% theta1=3*t^2+t;
% theta2=t;
% theta3=t^3+2*t;
% theta4=t;
% theta5=t;
% theta6=sin(t);
% 
% trajectory=[theta1 theta2 theta3 theta4 theta5 theta6];
% velocity=diff(trajectory,t);
% acceleration=diff(velocity,t);
% 
% time=1:0.1:10;
% taw=M*thetadd'+V+G;
% torque=subs(tao,[theta,thetad,thetadd,L1,L2,L3,L4],[trajectory,velocity,acceleration,l1,l2,l3,l4]);
% torque_time=subs(torque,t,time);
% 
% for i=1:6
% subplot(3,2,i);
%  plot(time,torque_time(i,:),'LineWidth',1);
%  grid on;
%  xlabel('time (s)');
%  ylabel(['Torque ',num2str(i),' (N.m)']);
% end






