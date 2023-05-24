function UR5_forward_test()
disp('Program started');
sim=remApi('remoteApi');
sim.simxFinish(-1);
%开始通信连接
clientID=sim.simxStart('127.0.0.1',19999,true,true,2000,5)
if clientID<0
    disp('Failed connecting to remote API server. Exiting.');
    sim.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n',clientID);
%开始仿真
sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot_wait);
%初始化所需要被操作的对象
h=UR5_Init(sim,clientID);
pause(.2);
disp('Satrting robot');
%获取关节位置
for i=1:6
    [res,pos(i)]=sim.simxGetJointPosition(clientID,h.Joints(i),sim.simx_opmode_buffer);%计算末端位姿
end
theta=pos
T=UR5_forward_kinematics(pos)
pause(2);
res=sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot);%仿真结束
end