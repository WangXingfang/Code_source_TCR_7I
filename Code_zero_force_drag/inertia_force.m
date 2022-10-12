function H = inertia_force(wx,wy,wz,dwx,dwy,dwz,ax,ay,az)
%UNTITLED6 此处显示有关此函数的摘要
%   此处显示详细说明
H(:,7:9)=Cross_S(wx,wy,wz)^2+Cross_S(dwx,dwy,dwz);
H(:,10)=[ax;ay;az];
end
