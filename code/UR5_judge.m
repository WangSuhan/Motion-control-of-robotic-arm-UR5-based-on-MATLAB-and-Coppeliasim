function result=UR5_judge(theta)
if theta<-pi+0.00001
    theta=theta+2*pi;
elseif theta>pi-0.00001
    theta=theta-2*pi;
end
result=theta;
end