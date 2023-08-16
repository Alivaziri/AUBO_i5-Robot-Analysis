clc;clear;close all

%%  FORWARD KINEMATICS

syms a alpha theta d 


T=[cos(theta) -sin(theta) 0 a
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d
    sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d
    0 0 0 1];

syms  t1 t2 t3 t4 t5 t6 l1 l2 l3 l4 l5 l6
T10=subs(T,[alpha,a,d,theta],[0,0,0,t1]);
T21=subs(T,[alpha,a,d,theta],[pi/2,0,l1, t2 + pi/2]);
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

%% JACOBIAN

z(1:3,1)=([T10(1,3); T10(2,3) ;T10(3,3)]);
z(1:3,2)=([T20(1,3); T20(2,3) ;T20(3,3)]);
z(1:3,3)=[T30(1,3); T30(2,3) ;T30(3,3)];
z(1:3,4)=[T40(1,3); T40(2,3) ;T40(3,3)];
z(1:3,5)=[T50(1,3); T50(2,3) ;T50(3,3)];
z(1:3,6)=[T60(1,3); T60(2,3) ;T60(3,3)];

o(1:3,1)=[T10(1,4); T10(2,4) ;T10(3,4)];
o(1:3,2)=[T20(1,4); T20(2,4) ;T20(3,4)];
o(1:3,3)=[T30(1,4); T30(2,4) ;T30(3,4)];
o(1:3,4)=[T40(1,4); T40(2,4) ;T40(3,4)];
o(1:3,5)=[T50(1,4); T50(2,4) ;T50(3,4)];
o(1:3,6)=[T60(1,4); T60(2,4) ;T60(3,4)];
oe=[T70(1,4); T70(2,4) ;T70(3,4)];

syms a
jv(1:3,1:6)=a;
jw(1:3,1:6)=a;

for i=1:6
    jv(1:3,i)=simplify(cross(z(1:3,i),(oe-o(1:3,i))));
    jw(1:3,i)=z(1:3,i);
end

J=[jv;jw];
J=simplify(J);
RANK=rank(J);
Jd = diff(J,t);


%% SINGULARITY ANALYSIS

% determinant=simplify(det(J));
