function A = Torque_A(wx,wy,wz,dwx,dwy,dwz,ax,ay,az)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
A(:,1:6)=Cross2_K(dwx,dwy,dwz)+Cross_S(wx,wy,wz)*Cross2_K(wx,wy,wz);
A(:,7:9)=Cross_S(-ax,-ay,-az);
A(:,10)=zeros(3,1);
end