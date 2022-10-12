clear ; clc; close all;
load I_table
g=9.80665; %重力加速度: m/s^2
%关节信息
Number=7;%连杆数
%L=[0.4,0.4]; %杆长/m
% i+1坐标系原点在i坐标系下的表示
P = 10^(-3)*[  0,   0,     0; 
	   0,   0,     0;
       0,   449.5, 0;
       30,  0,     0;
      -30,  449.5, 0;
       0,   0,     0;
       0,   0,     0;]';
%d=[0.2,0.2]; %质心距关节位置
% 质心在关节坐标系下的表示, unit: m
PC = 10^(-3)*I_table(2:4,:);
%alpha参数,unit:rad
alpha=[0,pi/2,-pi/2,pi/2,-pi/2,pi/2,-pi/2];
m=I_table(1,:); %重量, unit: kg
% 杆件在关节坐标系下的惯性张量 unit: kg*m^2
%......连杆数增加后更改IC内存，增加转动惯量数据
IC=cell(1,Number);
for i=1:Number
    IC{i}=10^(-6)*[I_table(5,i), -I_table(8,i), -I_table(9,i);
                  -I_table(8,i),  I_table(6,i), -I_table(10,i);
                  -I_table(9,i), -I_table(10,i), I_table(7,i)];
end
%......
%基座信息
% 基座角速度为0
omiga_v0 = [0; 0; 0];
% 基座角加速度为0
omiga_a0 = [0; 0; 0];
%重力因素
dv_0=[0 0 g]';%重力等效成向上为g的加速度
%迭代初始化
temp=0.001;%步长
tend=2; %结束时间
t_cont=0;%计数
%循环开始
for t=0:temp:tend
    t_cont=t_cont+1;
    %直接求导计算运动学参数
%    theta=[-pi/2*cos(pi*t) -pi/2*cos(pi*t) 0 0 0 0 0];
%    d_theta=diff(theta);
%    dd_theta=diff(theta,2);

%    theta = [-pi/2*cos(pi*t) -pi/2*cos(pi*t) 0 0 0 0 0];
%    d_theta=diff(theta);
%    dd_theta=diff(theta,2);
     %前向差分求运动学参数
    theta =[0 0 0 pi/2 pi/2*t 0 0];%设置theta相关函数,规划运动轨迹
    d_theta = [0 0 0 0 pi/2 0 0];
    dd_theta = [0 0 0 0 0 0 0]; 
     %保存每次迭代的theta值
    for i=1:Number
    theta_cont(i,t_cont) = theta(i);
    end
%{
    if t==0
        %初始关节角
        %根据初始时刻状态确定参数值
       d_theta = [-pi/2, -pi/2,0,0,0,0,0];
       dd_theta = [0, 0,0,0,0,0,0]; 
    else if t==temp
            d_theta = ( theta-theta_cont(:,t_cont-1) )/temp;%前向差分求d_theta
            dd_theta = [0, 0,0,0,0,0,0];
        else        
        d_theta = ( theta-theta_cont(:,t_cont-1) )/temp;%向前差分求d_theta
        dd_theta = ( theta-2*theta_cont(:,t_cont-1)+theta_cont(:,t_cont-2) )/temp^2;%前向差分求dd_theta
        end
    end
%}


    % 获取杆件间的旋转矩阵(i+1在i下的表示) i上 i+1下
    R=cell(1,Number);
    R0=cell(1,Number);    
    for i=1:Number
    R0{i} = [1,            0,             0;
             0,cos(alpha(i)),-sin(alpha(i));
             0,sin(alpha(i)), cos(alpha(i))];
    R{i} = [cos(theta(i)),  -sin(theta(i)), 0.0; 
            sin(theta(i)),  cos(theta(i)),  0.0; 
            0.0,            0.0,            1.0];
    R{i} = R0{i}*R{i};
    end

    %......增加连杆后在此增加连杆旋转矩阵
    %......
    %牛顿-欧拉方程
    Z=[0 0 1]';%关节坐标旋转轴方向向量
    %外推
    %连杆1
	    omiga_v(:,1)=R{1}\omiga_v0+d_theta(1)*Z;
	    omiga_a(:,1)=R{1}\omiga_a0+cross(R{1}\omiga_v0, d_theta(1)*Z) +dd_theta(1)*Z;
	    dv(:,1)=R{1}\( cross(omiga_a0, P(:,1) )+cross( omiga_v0, cross( omiga_v0, P(:,1) ) )+dv_0 );
	    dv_c(:,1)=cross( omiga_a(:,1),PC(:,1) )+cross( omiga_v(:,1),cross( omiga_v(:,1),PC(:,1) ) )+dv(:,1);
	    F(:,1)=dv_c(:,1)*m(1);
	    N(:,1)=IC{1}*omiga_a(:,1)+cross(omiga_v(:,1),IC{1}*omiga_v(:,1));
    %连杆2~Number
    for i=2:Number
	    omiga_v(:,i)=R{i}\omiga_v(:,i-1)+d_theta(i)*Z;
	    omiga_a(:,i)=R{i}\omiga_a(:,i-1)+cross( R{i}\omiga_v(:,i-1), d_theta(i)*Z ) +dd_theta(i)*Z;
	    dv(:,i)=R{i}\( cross( omiga_a(:,i-1), P(:,i) )+cross( omiga_v(:,i-1), cross(omiga_v(:,i-1),P(:,i) ) )+dv(:,i-1) );
	    dv_c(:,i)=cross( omiga_a(:,i),PC(:,i) )+cross( omiga_v(:,i),cross( omiga_v(:,i),PC(:,i) ) )+dv(:,i);
	    F(:,i)=dv_c(:,i)*m(i);
	    N(:,i)=IC{i}*omiga_a(:,i)+cross( omiga_v(:,i),IC{i}*omiga_v(:,i) );
    end
    %内推
    %连杆Number
	    f(1:3,Number)=F(:,Number);
	    n(1:3,Number)=N(:,Number)+cross(PC(:,Number), F(:,Number));
    %连杆1~Number-1
    for i=Number-1:1
	    f(1:3,i)=R{i+1}*f(1:3,i+1)+F(:,i);
	    n(1:3,i)=N(:,i)+R{i+1}*n(1:3,i+1)+cross( PC(:,i), F(:,i) )+cross( P(:,i+1), R{i+1}*f(1:3,i+1) );
    end
    time(t_cont)=t;%计时
    for i=1:Number
    	tao(t_cont,i)=n(:,i)'*Z;
    end
end

%为输出力矩结果表格x.t文件存入数据
Torquechart(:,1)=time(1,:)';%时间
for i=1:Number
   	Torquechart(:,i+1)=tao(:,i);
end

t=0:temp:tend;

for i=1:Number
    subplot(2,Number,i)
    plot(t,tao(:,i))
    xlabel('t')
    ylabel('tao')
end
for i=1:Number
    subplot(2,Number,Number+i)
    plot(t,theta_cont(i,:))
    xlabel('t')
    ylabel('theta')
end
save afile.txt -ascii Torquechart