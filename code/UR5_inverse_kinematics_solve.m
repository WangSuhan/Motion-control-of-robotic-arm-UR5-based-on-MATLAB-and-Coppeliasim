function theta_out=UR5_inverse_kinematics_solve(T,theta)
%T为目标点的位姿矩阵，theta为各关节当前的角度
nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
%DH参数设置
%DH矩阵第二列为连杆距离，第三列为连杆长度，第四列为连杆扭角
DH=[-pi/2 0.0892 0 0;
    pi/2 0 0 pi/2;
    0 0 0.425 0;
    -pi/2 0.1093 0.392 0;
    0 0.09475 0 -pi/2;
    0 0.0825 0 pi/2];
d1=DH(1,2);d2=DH(2,2);d3=DH(3,2);d4=DH(4,2);d5=DH(5,2);d6=DH(6,2);
a0=DH(1,3);a1=DH(2,3);a2=DH(3,3);a3=DH(4,3);a4=DH(5,3);a5=DH(6,3);
alpha0=DH(1,4);alpha1=DH(2,4);alpha2=DH(3,4);alpha3=DH(4,4);alpha4=DH(5,4);alpha5=DH(6,4);
%求解theta1,最终得到两个解
m1=d6*ay-py;
n1=d6*ax-px;
%判断是否奇异
if m1^2+n1^2-d4^2<1e-6
    fprintf('肩关节theta1奇异，请检查奇异条件\n')
    theta1_1=100;
    theta1_2=100;
else
    theta1_1=atan2(-m1,-n1)-atan2(-d4,+sqrt(m1^2+n1^2-d4^2));
    theta1_2=atan2(-m1,-n1)-atan2(-d4,-sqrt(m1^2+n1^2-d4^2));
    theta1_1=UR5_judge(theta1_1);
    theta1_2=UR5_judge(theta1_2);
end
%求解theta5,由theta1决定，最终得到4个解
theta5_1=acos(ax*sin(theta1_1)-ay*cos(theta1_1));
theta5_2=-acos(ax*sin(theta1_1)-ay*cos(theta1_1));
theta5_3=acos(ax*sin(theta1_2)-ay*cos(theta1_2));
theta5_4=-acos(ax*sin(theta1_2)-ay*cos(theta1_2));
theta5_1=UR5_judge(theta5_1);
theta5_2=UR5_judge(theta5_2);
theta5_3=UR5_judge(theta5_3);
theta5_4=UR5_judge(theta5_4);
%求解theta6,由theta1,theta5决定，最终得到四个解
m6_1=ny*cos(theta1_1)-nx*sin(theta1_1);
n6_1=oy*cos(theta1_1)-ox*sin(theta1_1);
m6_2=ny*cos(theta1_2)-nx*sin(theta1_2);
n6_2=oy*cos(theta1_2)-ox*sin(theta1_2);
theta6_1=atan2(m6_1,n6_1)-atan2(sin(theta5_1),0);
theta6_2=atan2(m6_1,n6_1)-atan2(sin(theta5_2),0);
theta6_3=atan2(m6_2,n6_2)-atan2(sin(theta5_3),0);
theta6_4=atan2(m6_2,n6_2)-atan2(sin(theta5_4),0);
theta6_1=UR5_judge(theta6_1);
theta6_2=UR5_judge(theta6_2);
theta6_3=UR5_judge(theta6_3);
theta6_4=UR5_judge(theta6_4);
%求解theta2+3+4，由theta1,theta5决定，最终得到四个解
if abs(theta5_1)<1e-6
    fprintf('腕关节theta5_1奇异，请检查theta5值是否为0 \n')
end
if abs(theta5_2)<1e-6
    fprintf('腕关节theta5_2奇异，请检查theta5值是否为0 \n')
end
if abs(theta5_3)<1e-6
    fprintf('腕关节theta5_3奇异，请检查theta5值是否为0 \n')
end
if abs(theta5_4)<1e-6
    fprintf('腕关节theta5_4奇异，请检查theta5值是否为0 \n')
end
m234_1=ax*cos(theta1_1)+ay*sin(theta1_1);
m234_2=ax*cos(theta1_2)+ay*sin(theta1_2);
theta234_1=atan2(az/sin(theta5_1),m234_1/sin(theta5_1));
theta234_2=atan2(az/sin(theta5_2),m234_1/sin(theta5_2));
theta234_3=atan2(az/sin(theta5_3),m234_2/sin(theta5_3));
theta234_4=atan2(az/sin(theta5_4),m234_2/sin(theta5_4));
theta234_1=UR5_judge(theta234_1);
theta234_2=UR5_judge(theta234_2);
theta234_3=UR5_judge(theta234_3);
theta234_4=UR5_judge(theta234_4);
%求解theta3,由theta1,theta234,theta5决定，最终得到八个解
m3_1=px*cos(theta1_1)+py*sin(theta1_1)+d5*sin(theta234_1)-d6*sin(theta5_1)*cos(theta234_1);
n3_1=pz-d1-d5*cos(theta234_1)-d6*sin(theta5_1)*sin(theta234_1);
m3_2=px*cos(theta1_1)+py*sin(theta1_1)+d5*sin(theta234_2)-d6*sin(theta5_2)*cos(theta234_2);
n3_2=pz-d1-d5*cos(theta234_2)-d6*sin(theta5_2)*sin(theta234_2);
m3_3=px*cos(theta1_2)+py*sin(theta1_2)+d5*sin(theta234_3)-d6*sin(theta5_3)*cos(theta234_3);
n3_3=pz-d1-d5*cos(theta234_3)-d6*sin(theta5_3)*sin(theta234_3);
m3_4=px*cos(theta1_2)+py*sin(theta1_2)+d5*sin(theta234_4)-d6*sin(theta5_4)*cos(theta234_4);
n3_4=pz-d1-d5*cos(theta234_4)-d6*sin(theta5_4)*sin(theta234_4);
if(((m3_1^2+n3_1^2)<(a2-a3)^2)||((m3_1^2+n3_1^2)>(a2+a3)^2))
    fprintf('肘关节theta3_1或theta3_2奇异，请检查奇异条件\n')
    theta3_1=100;
    theta3_2=100;
else
    theta3_1=acos((m3_1^2+n3_1^2-a2^2-a3^2)/(2*a2*a3));
    theta3_2=-acos((m3_1^2+n3_1^2-a2^2-a3^2)/(2*a2*a3));
    theta3_1=UR5_judge(theta3_1);
    theta3_2=UR5_judge(theta3_2);
end
if(((m3_2^2+n3_2^2)<(a2-a3)^2)||((m3_2^2+n3_2^2)>(a2+a3)^2))
    fprintf('肘关节theta3_3或theta3_4奇异，请检查奇异条件\n')
    theta3_3=100;
    theta3_4=100;
else
    theta3_3=acos((m3_2^2+n3_2^2-a2^2-a3^2)/(2*a2*a3));
    theta3_4=-acos((m3_2^2+n3_2^2-a2^2-a3^2)/(2*a2*a3));
    theta3_3=UR5_judge(theta3_3);
    theta3_4=UR5_judge(theta3_4);
end
if(((m3_3^2+n3_3^2)<(a2-a3)^2)||((m3_3^2+n3_3^2)>(a2+a3)^2))
    fprintf('肘关节theta3_5或theta3_6奇异，请检查奇异条件\n')
    theta3_5=100;
    theta3_6=100;
else
    theta3_5=acos((m3_3^2+n3_3^2-a2^2-a3^2)/(2*a2*a3));
    theta3_6=-acos((m3_3^2+n3_3^2-a2^2-a3^2)/(2*a2*a3));
    theta3_5=UR5_judge(theta3_5);
    theta3_6=UR5_judge(theta3_6);
end
if(((m3_4^2+n3_4^2)<(a2-a3)^2)||((m3_4^2+n3_4^2)>(a2+a3)^2))
    fprintf('肘关节theta3_7或theta3_8奇异，请检查奇异条件\n')
    theta3_7=100;
    theta3_8=100;
else
    theta3_7=acos((m3_4^2+n3_4^2-a2^2-a3^2)/(2*a2*a3));
    theta3_8=-acos((m3_4^2+n3_4^2-a2^2-a3^2)/(2*a2*a3));
    theta3_7=UR5_judge(theta3_7);
    theta3_8=UR5_judge(theta3_8);
end
%求解theta2,由theta3决定，最终得到8个解
s2_1=((a3*cos(theta3_1)+a2)*n3_1-a3*sin(theta3_1)*m3_1)/(a2^2+a3^2+2*a2*a3*cos(theta3_1));
s2_2=((a3*cos(theta3_2)+a2)*n3_1-a3*sin(theta3_2)*m3_1)/(a2^2+a3^2+2*a2*a3*cos(theta3_2));
s2_3=((a3*cos(theta3_3)+a2)*n3_2-a3*sin(theta3_3)*m3_2)/(a2^2+a3^2+2*a2*a3*cos(theta3_3));
s2_4=((a3*cos(theta3_4)+a2)*n3_2-a3*sin(theta3_4)*m3_2)/(a2^2+a3^2+2*a2*a3*cos(theta3_4));
s2_5=((a3*cos(theta3_5)+a2)*n3_3-a3*sin(theta3_5)*m3_3)/(a2^2+a3^2+2*a2*a3*cos(theta3_5));
s2_6=((a3*cos(theta3_6)+a2)*n3_3-a3*sin(theta3_6)*m3_3)/(a2^2+a3^2+2*a2*a3*cos(theta3_6));
s2_7=((a3*cos(theta3_7)+a2)*n3_4-a3*sin(theta3_7)*m3_4)/(a2^2+a3^2+2*a2*a3*cos(theta3_7));
s2_8=((a3*cos(theta3_8)+a2)*n3_4-a3*sin(theta3_8)*m3_4)/(a2^2+a3^2+2*a2*a3*cos(theta3_8));
c2_1=((a3*cos(theta3_1)+a2)*m3_1+a3*sin(theta3_1)*n3_1)/(a2^2+a3^2+2*a2*a3*cos(theta3_1));
c2_2=((a3*cos(theta3_2)+a2)*m3_1+a3*sin(theta3_2)*n3_1)/(a2^2+a3^2+2*a2*a3*cos(theta3_2));
c2_3=((a3*cos(theta3_3)+a2)*m3_2+a3*sin(theta3_3)*n3_2)/(a2^2+a3^2+2*a2*a3*cos(theta3_3));
c2_4=((a3*cos(theta3_4)+a2)*m3_2+a3*sin(theta3_4)*n3_2)/(a2^2+a3^2+2*a2*a3*cos(theta3_4));
c2_5=((a3*cos(theta3_5)+a2)*m3_3+a3*sin(theta3_5)*n3_3)/(a2^2+a3^2+2*a2*a3*cos(theta3_5));
c2_6=((a3*cos(theta3_6)+a2)*m3_3+a3*sin(theta3_6)*n3_3)/(a2^2+a3^2+2*a2*a3*cos(theta3_6));
c2_7=((a3*cos(theta3_7)+a2)*m3_4+a3*sin(theta3_7)*n3_4)/(a2^2+a3^2+2*a2*a3*cos(theta3_7));
c2_8=((a3*cos(theta3_8)+a2)*m3_4+a3*sin(theta3_8)*n3_4)/(a2^2+a3^2+2*a2*a3*cos(theta3_8));
if(theta3_1==100&&theta3_2==100)
    theta2_1=100;
    theta2_2=100
else
    theta2_1=atan2(s2_1,c2_1);
    theta2_2=atan2(s2_2,c2_2);
    theta2_1=UR5_judge(theta2_1);
    theta2_2=UR5_judge(theta2_2);
end
if(theta3_3==100&&theta3_4==100)
    theta2_3=100;
    theta2_4=100
else
    theta2_3=atan2(s2_3,c2_3);
    theta2_4=atan2(s2_4,c2_4);
    theta2_3=UR5_judge(theta2_3);
    theta2_4=UR5_judge(theta2_4);
end
if(theta3_5==100&&theta3_6==100)
    theta2_5=100;
    theta2_6=100
else
    theta2_5=atan2(s2_5,c2_5);
    theta2_6=atan2(s2_6,c2_6);
    theta2_6=UR5_judge(theta2_6);
    theta2_6=UR5_judge(theta2_6);
end
if(theta3_7==100&&theta3_8==100)
    theta2_7=100;
    theta2_8=100
else
    theta2_7=atan2(s2_7,c2_7);
    theta2_8=atan2(s2_8,c2_8);
    theta2_7=UR5_judge(theta2_7);
    theta2_8=UR5_judge(theta2_8);
end
%求解theta4,由theta234,theta2,theta3决定，最终得到8个解
theta4_1=theta234_1-theta2_1-theta3_1;
theta4_2=theta234_1-theta2_2-theta3_2;
theta4_3=theta234_2-theta2_3-theta3_3;
theta4_4=theta234_2-theta2_4-theta3_4;
theta4_5=theta234_3-theta2_5-theta3_5;
theta4_6=theta234_3-theta2_6-theta3_6;
theta4_7=theta234_4-theta2_7-theta3_7;
theta4_8=theta234_4-theta2_8-theta3_8;
theta4_1=UR5_judge(theta4_1);
theta4_2=UR5_judge(theta4_2);
theta4_3=UR5_judge(theta4_3);
theta4_4=UR5_judge(theta4_4);
theta4_5=UR5_judge(theta4_5);
theta4_6=UR5_judge(theta4_6);
theta4_7=UR5_judge(theta4_7);
theta4_8=UR5_judge(theta4_8);
%最终得到8组组合解
theta_T=[theta1_1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1;
    theta1_1 theta2_2 theta3_2 theta4_2 theta5_1 theta6_1;
    theta1_1 theta2_3 theta3_3 theta4_3 theta5_2 theta6_2;
    theta1_1 theta2_4 theta3_4 theta4_4 theta5_2 theta6_2;
    theta1_2 theta2_5 theta3_5 theta4_5 theta5_3 theta6_3;
    theta1_2 theta2_6 theta3_6 theta4_6 theta5_3 theta6_3;
    theta1_2 theta2_7 theta3_7 theta4_7 theta5_4 theta6_4;
    theta1_2 theta2_8 theta3_8 theta4_8 theta5_4 theta6_4]
%逆解结果带入正运动学求解函数得到位姿矩阵
size_theta_T=size(theta_T);
theta_T1=theta_T;
m=1;
for i=1:size_theta_T(1)
    if(theta_T(i,1)==100||theta_T(i,3)==100)
        fprintf('第%d组解为奇异解1\n',i);
    else
        T0n_kine=UR5_forward_kinematics(theta_T(i,:));
        delta_T=T0n_kine-T;
        delta_T=delta_T(1:3,:);
        if abs(delta_T)>1e-6
            fprintf('第%d组解为奇异解1\n',i);
        else
            fprintf('第%d组解为有效解1\n',i);
            theta_T1(m,:)=theta_T(i,:);
            m=m+1;
        end
    end
end
if((m-1)==0)
    fprintf('超出工作范围，请检查物体摆放位置\n',i);
else
    theta_all=theta_T1(1:m-1,:);
    size_theta_all=size(theta_all);
    %从符合奇异条件的逆解中寻找一组最优解，作为逆解函数返回值
    %关节空间当前的转角为theta，根据指定的theta，将合格的每一组解与theta对比，取其差值的平方和最小的一组解为最优解，保证所有关节运动距离最小。
    sum_abs_delta=zeros(size_theta_all(1),1);
    for i=1:size_theta_all(1)
        abs_delta_theta_all=abs(theta_all(i,:)-theta);
        sum_abs_delta_theta_all=sum(abs_delta_theta_all.*abs_delta_theta_all,2);
        sum_abs_delta(i,1)=sum_abs_delta_theta_all;
    end
    min_sum_abs_delta=min(sum_abs_delta(:));
    [row,rank]=find(sum_abs_delta==min_sum_abs_delta);
    theta_out=theta_all(row,:);
end
end