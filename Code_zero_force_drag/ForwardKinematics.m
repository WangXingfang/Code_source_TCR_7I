function T07 = ForwardKinematics(angle)%#codegen
%% 正运动学2
%功能----------根据7个关节角计算末端位姿
%输入----------七个关节角1x7
%输出----------末端位姿4x4
T01 = CoordinateTrans(0,0,angle(1),0);       %CoordinateTrans(alpha_0,a_0,theta_1,d_1)是自己写的函数，MDH的坐标系转换公式
T12 = CoordinateTrans(pi/2,0,angle(2),0);
T23 = CoordinateTrans(-pi/2,0,angle(3),449.5);
T34 = CoordinateTrans(pi/2,30,angle(4),0);
T45 = CoordinateTrans(-pi/2,-30,angle(5),449.5);
T56 = CoordinateTrans(pi/2,0,angle(6),0);
T67 = CoordinateTrans(-pi/2,0,angle(7),0);
T07 = T01*T12*T23*T34*T45*T56*T67;
end