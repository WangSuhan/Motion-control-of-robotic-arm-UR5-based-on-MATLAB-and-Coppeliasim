function T=UR5_forward_kinematics(q)
% DH参数矩阵
DH=[-pi/2 0.0892 0 0;
    pi/2 0 0 pi/2;
    0 0 0.425 0;
    -pi/2 0.1093 0.392 0;
    0 0.09475 0 -pi/2;
    0 0.0825 0 pi/2];
%q为连杆转角
%DH矩阵第二列为连杆距离,第三列为连杆长度，第四列连杆扭角
%Tij为第i连杆相对于第j杆的位姿
T10=zeros(4,4);T21=zeros(4,4);T32=zeros(4,4);T43=zeros(4,4);T54=zeros(4,4);T65=zeros(4,4);
T00={T10,T21,T32,T43,T54,T65};
for i=1:6
    T00{1,i}=[cos(q(i)) -sin(q(i)) 0 DH(i,3);
        sin(q(i))*cos(DH(i,4)) cos(q(i))*cos(DH(i,4)) -sin(DH(i,4)) -DH(i,2)*sin(DH(i,4));
        sin(q(i))*sin(DH(i,4)) cos(q(i))*sin(DH(i,4)) cos(DH(i,4)) DH(i,2)*cos(DH(i,4));
        0 0 0 1];
end
T10=T00{1,1};T21=T00{1,2};T32=T00{1,3};T43=T00{1,4};T54=T00{1,5};T65=T00{1,6};
T20=T10*T21;
T30=T20*T32;
T40=T30*T43;
T50=T40*T54;
T60=T50*T65;
T=T60;
end