function Tout =CoordinateTrans_szy(alpha0,a0,theta1,d1)%#codegen
%% ���˶�ѧ
%����----------����MDH�Ĳ�����������������ϵת�ƾ���
%����----------MDH�Ĳ���
%���----------��������ϵת�ƾ���4x4
ca0 = cos(alpha0);
sa0 = sin(alpha0);

	if(abs(alpha0 - pi/2) < 10e-6)  %alpha0 ����Ϊ pi/2
		ca0 = 0;
		sa0 = 1;
    elseif (abs(alpha0) < 10e-6) %alpha0 ����Ϊ0
		ca0 = 1;
		sa0 = 0;
	elseif (abs(alpha0 + pi/2) < 10e-6) %alpha0 ����Ϊ -pi/2
		ca0 = 0;
		sa0 = -1;
	elseif (abs(alpha0 - pi) < 10e-6) %alpha0 ����Ϊ pi
		ca0 = -1;
		sa0 = 0;
	elseif (abs(alpha0 + pi) < 10e-6) %alpha0 ����Ϊ -pi
		ca0 = -1;
		sa0 = 0;
	end
Tout = [ cos(theta1)             -sin(theta1)               0             a0 ;
                    sin(theta1)*ca0         cos(theta1)*ca0            -sa0          -sa0*d1;
                    sin(theta1)*sa0         cos(theta1)*sa0            ca0           ca0*d1;
                    0                         0                        0             1             ;];
end