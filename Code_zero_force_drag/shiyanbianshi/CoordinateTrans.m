function coordinateTrans =CoordinateTrans(alpha0,a0,theta1,d1)%#codegen
%% 正运动学
%功能----------依据MDH四参数，计算相邻坐标系转移矩阵
%输入----------MDH四参数
%输出----------相邻坐标系转移矩阵4x4
coordinateTrans = [ cos(theta1)               -sin(theta1)               0              a0 ;
                    sin(theta1)*cos(alpha0)   cos(theta1)*cos(alpha0)    -sin(alpha0)   -sin(alpha0)*d1;
                    sin(theta1)*sin(alpha0)   cos(theta1)*sin(alpha0)    cos(alpha0)    cos(alpha0)*d1;
                    0                         0                          0              1             ;];
end