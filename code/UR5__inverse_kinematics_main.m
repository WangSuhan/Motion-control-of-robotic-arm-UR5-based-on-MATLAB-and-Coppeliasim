clear all;
disp('Program started');
currentJoints=zeros(1,6);%初始化关节角度反馈值
sim=remApi('remoteApi');%加载Coppeliasim远程库
sim.simxFinish(-1);%关闭所有sim通信；
%定义sim中各关节句柄名称
JointNames={'UR5_joint1','UR5_joint2','UR5_joint3','UR5_joint4','UR5_joint5','UR5_joint6'};
res1=1;res2=1;res3=1;%返回值有效性判断
%建立与Coppeliasim通信
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
    %启动sim的场景仿真
    res1=sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot_wait);
    %读取各个关节句柄值
    for i=1:6
        [res1,sixJoints(i)]=sim.simxGetObjectHandle(clientID,JointNames{i},sim.simx_opmode_oneshot_wait);
    end
    %获得sim中相应名称的对象句柄值，'target'为抓取目标对象
    [res1,handle_target_0]=sim.simxGetObjectHandle(clientID,'Target',sim.simx_opmode_blocking);
    [res1,handle_base_o]=sim.simxGetObjectHandle(clientID,'Base',sim.simx_opmode_blocking);
    if(sim.simxGetConnectionId(clientID)~=-1)%判断连接是否正常
        while(res2==-1||res3==1)
[res2,T_targetPosition]=sim.simxGetObjectPosition(clientID,handle_target_0,handle_base_o,sim.simx_opmode_streaming);
[res3,T_targetOrientation]=sim.simxGetObjectOrientation(clientID,handle_target_0,handle_base_o,sim.simx_opmode_streaming);
        end
        T_targetPosition;
        T_targetOrientation;
        T_targetOrientation=double(T_targetOrientation);
        T_targetPosition=double(T_targetPosition);
        %转换为T矩阵中的位置向量
        target2baseposition=transl(T_targetPosition);
        %转换为T矩阵中的姿态向量，获得的原始姿态是用RPY（roll,pitch,yaw)表示的
        target2baseOrientation=trotx(T_targetOrientation(1))*troty(T_targetOrientation(2))*trotz(T_targetOrientation(3));
        %只有位置的矩阵和只有姿态的矩阵拼起来，得到目标点相对于基坐标系的位姿
        target2basePos=target2baseposition*target2baseOrientation
        %读取Coppeliasim中UR5机器人6个关节当前的角度值
        for i=1:6
            [returnCode,currentJoints(i)]=sim.simxGetJointPosition(clientID,sixJoints(i),sim.simx_opmode_oneshot_wait);
        end
% currentJoints=[-pi/2 pi/2 0 -pi/2 0 0];
        %利用逆运动学函数优化求解6个关节角度值
        nextJoints=UR5_inverse_kinematics_solve(target2basePos,currentJoints);
%         %数值迭代求解
%         mdl_puma560;%使用robotics toolbook中的模型计算
%         nextJoints=p560.ikine(target2basePos,currentJoints);%数值迭代求逆解；
%         %运动到目标位姿，一步快速到位
%         for i=1:6
%             res=sim.simxSetJointTargetPosition(clientID,sixJoints(i),nextJoints(i),sim.simx_opmode_oneshot);
%         end
%运动到目标位姿，多步逐渐到位
A1=currentJoints;
A2=nextJoints;
timestep=.5;
time=20;
N_step=time/timestep;
[joint1,joint1d,joint1dd]=tpoly(A1(1),A2(1),N_step);
[joint2,joint2d,joint2dd]=tpoly(A1(2),A2(2),N_step);
[joint3,joint3d,joint3dd]=tpoly(A1(3),A2(3),N_step);
[joint4,joint4d,joint4dd]=tpoly(A1(4),A2(4),N_step);
[joint5,joint5d,joint5dd]=tpoly(A1(5),A2(5),N_step);
[joint6,joint6d,joint6dd]=tpoly(A1(6),A2(6),N_step);
[joint12,joint12d,joint12dd]=tpoly(A2(1),A1(1),N_step);
[joint22,joint22d,joint22dd]=tpoly(A2(2),A1(2),N_step);
[joint32,joint32d,joint32dd]=tpoly(A2(3),A1(3),N_step);
[joint42,joint42d,joint42dd]=tpoly(A2(4),A1(4),N_step);
[joint52,joint52d,joint52dd]=tpoly(A2(5),A1(5),N_step);
[joint62,joint62d,joint62dd]=tpoly(A2(6),A1(6),N_step);
for i=1:N_step
    sim.simxPauseCommunication(clientID,0);%开始通信
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(1),joint1(i),sim.simx_opmode_oneshot);
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(2),joint2(i),sim.simx_opmode_oneshot);
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(3),joint3(i),sim.simx_opmode_oneshot);
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(4),joint4(i),sim.simx_opmode_oneshot);
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(5),joint5(i),sim.simx_opmode_oneshot);
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(6),joint6(i),sim.simx_opmode_oneshot);
    sim.simxPauseCommunication(clientID,1);%终止通信
    pause(.02)
end
sim.simxPauseCommunication(clientID,0);
pause(2);
for i=1:N_step
    sim.simxPauseCommunication(clientID,0);%开始通信
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(1),joint12(i),sim.simx_opmode_oneshot);
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(2),joint22(i),sim.simx_opmode_oneshot);
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(3),joint32(i),sim.simx_opmode_oneshot);
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(4),joint42(i),sim.simx_opmode_oneshot);
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(5),joint52(i),sim.simx_opmode_oneshot);
    res=sim.simxSetJointTargetPosition(clientID,sixJoints(6),joint62(i),sim.simx_opmode_oneshot);
    sim.simxPauseCommunication(clientID,1);%终止通信
    pause(.02)
end
%多步到位结束
sim.simxPauseCommunication(clientID,0);
    end
    pause(2);
    sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot);
end