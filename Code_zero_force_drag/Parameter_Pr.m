function Pr = Parameter_Pr(d,a,alpha)
%UNTITLED3 此处显示有关此函数的摘要
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
%     Ixx(i)=Ixx(i)+m(i)*(y(i)^2+z(i)^2);
%     Ixy(i)=Ixy(i)-m(i)*x(i)*y(i);
%     Ixz(i)=Ixx(i)-m(i)*x(i)*z(i);
%     Iyy(i)=Iyy(i)+m(i)*(x(i)^2+z(i)^2);
%     Iyz(i)=Iyz(i)-m(i)*y(i)*z(i);
%     Izz(i)=Izz(i)+m(i)*(x(i)^2+y(i)^2);
end
for i=7:-1:2
	Ixx(i-1)=Ixx(i-1)+Iyy(i)+2*d(i)*mz(i)+d(i)^2*m(i);
	Ixy(i-1)=Ixy(i-1)+a(i)*sind(alpha(i))*(mz(i)+d(i)*m(i));
	Ixz(i-1)=Ixz(i-1)-a(i)*cosd(alpha(i))*(mz(i)+d(i)*m(i));
	Iyy(i-1)=Iyy(i-1)+cosd(alpha(i))^2*Iyy(i)+2*d(i)*cosd(alpha(i))^2*mz(i)+(d(i)^2*cosd(alpha(i))^2+a(i)^2)*m(i);
	Iyz(i-1)=Iyz(i-1)+cosd(alpha(i))*sind(alpha(i))*(Iyy(i)+2*d(i)*mz(i)+d(i)^2*m(i));
	Izz(i-1)=Izz(i-1)+sind(alpha(i))^2*Iyy(i)+2*d(i)*sind(alpha(i))^2*mz(i)+(d(i)^2*sind(alpha(i))^2+a(i)^2)*m(i);
	mx(i-1)=mx(i-1)+a(i)*m(i);
	my(i-1)=my(i-1)-sind(alpha(i))*(mz(i)+d(i)*m(i));
	mz(i-1)=mz(i-1)+cosd(alpha(i))*(mz(i)+d(i)*m(i));
	m(i-1)=m(i-1)+m(i);
end
P=cell(1,7);
P{1}=[Izz(1)];
for i=7:-1:2
	P{i}=[Ixx(i)-Iyy(i) Ixy(i) Ixz(i) Iyz(i) Izz(i) mx(i), my(i)].'
end
    Pr=[P{1};P{2};P{3};P{4};P{5};P{6};P{7}];
end

