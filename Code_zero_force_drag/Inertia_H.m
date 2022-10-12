function Inertia_H(wx,wy,wz,dwx,dwy,dwz,ax,ay,az)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
H(:,1:6)=zeros(3,6);
H(:,7:9)=Cross_S(dwx,dwy,dwz)+Cross_S(wx,wy,wz)*Cross_S(wx,wy,wz);
H(:,10)=[ax,ay,az]';
end

