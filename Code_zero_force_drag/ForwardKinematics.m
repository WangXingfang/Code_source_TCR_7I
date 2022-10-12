function T07 = ForwardKinematics(angle)%#codegen
%% ���˶�ѧ2
%����----------����7���ؽڽǼ���ĩ��λ��
%����----------�߸��ؽڽ�1x7
%���----------ĩ��λ��4x4
T01 = CoordinateTrans(0,0,angle(1),0);       %CoordinateTrans(alpha_0,a_0,theta_1,d_1)���Լ�д�ĺ�����MDH������ϵת����ʽ
T12 = CoordinateTrans(pi/2,0,angle(2),0);
T23 = CoordinateTrans(-pi/2,0,angle(3),449.5);
T34 = CoordinateTrans(pi/2,30,angle(4),0);
T45 = CoordinateTrans(-pi/2,-30,angle(5),449.5);
T56 = CoordinateTrans(pi/2,0,angle(6),0);
T67 = CoordinateTrans(-pi/2,0,angle(7),0);
T07 = T01*T12*T23*T34*T45*T56*T67;
end