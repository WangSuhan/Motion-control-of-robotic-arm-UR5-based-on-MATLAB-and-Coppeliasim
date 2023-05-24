function handles=UR5_Init(sim,clientID)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
handles=struct('id',clientID);
Joints=[-1,-1,-1,-1,-1,-1];
%获取Joints的句柄
[res Joints(1)]=sim.simxGetObjectHandle(clientID,'UR5_joint1',sim.simx_opmode_oneshot_wait);
[res Joints(2)]=sim.simxGetObjectHandle(clientID,'UR5_joint2',sim.simx_opmode_oneshot_wait);
[res Joints(3)]=sim.simxGetObjectHandle(clientID,'UR5_joint3',sim.simx_opmode_oneshot_wait);
[res Joints(4)]=sim.simxGetObjectHandle(clientID,'UR5_joint4',sim.simx_opmode_oneshot_wait);
[res Joints(5)]=sim.simxGetObjectHandle(clientID,'UR5_joint5',sim.simx_opmode_oneshot_wait);
[res Joints(6)]=sim.simxGetObjectHandle(clientID,'UR5_joint6',sim.simx_opmode_oneshot_wait);
handles.Joints=Joints;
for i=1:6
    res=sim.simxGetJointPosition(clientID,Joints(i),sim.simx_opmode_streaming);
end
end