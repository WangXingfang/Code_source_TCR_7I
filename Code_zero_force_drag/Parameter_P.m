function P = Parameter_P(d,a,alpha)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
load I_table
Ixx=10^(-6)*I_table(5,:);
Ixy=10^(-6)*I_table(8,:);
Ixz=10^(-6)*I_table(9,:);
Iyy=10^(-6)*I_table(6,:);
Iyz=10^(-6)*I_table(10,:);
Izz=10^(-6)*I_table(7,:);
x=10^(-3)*I_table(2,:);
y=10^(-3)*I_table(3,:);
z=10^(-3)*I_table(4,:);
m=I_table(1,:);
for i=1:7
    mx(i)=m(i)*x(i);
    my(i)=m(i)*y(i);
    mz(i)=m(i)*z(i);
end
p=cell(1,7)
for i=1:7
    p{i}=[Ixx(i),Ixy(i),Ixz(i),Iyy(i),Iyz(i),Izz(i),mx(i),my(i),mz(i),m(i)];
end
P=[p{1},p{2},p{3},p{4},p{5},p{6},p{7}].';
end

