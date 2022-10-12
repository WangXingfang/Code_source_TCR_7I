function B = Cross_B(wx,wy,wz,dwx,dwy,dwz,ax,ay,az)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
% B(:,1:6)=zeros(3,6);
B(:,7:9)=[0 -wz wy;wz 0 -wx;-wy wx 0]^2+Cross_S(dwx,dwy,dwz);
 B(:,10)=[ax;ay;az];
end