clear ; clc; close all;
load I_table
g=10;
% g=9.80665; %重力加速度: m/s^2
%关节信息
Number=7;%连杆数
%L=[0.4,0.4]; %杆长/m
% i+1坐标系原点在i坐标系下的表示
P = [  0,   0,     0; 
	   0,   0,     0;
       0,   0.4495, 0;
       0.03,  0,     0;
      -0.03,  0.4495, 0;
       0,   0,     0;
       0,   0,     0;]';
%d=[0.2,0.2]; %质心距关节位置
% 质心在关节坐标系下的表示, unit: 
PC = 10^(-3)*I_table(2:4,:);
%alpha参数,unit:rad
alpha=[0,90,-90,90,-90,90,-90];
        %m=I_table(1,:); %重量, unit: kg
        % 杆件在关节坐标系下的惯性张量 unit: kg*m^2
        %......连杆数增加后更改IC内存，增加转动惯量数据
        %IC=cell(1,Number);
        %for i=1:Number
        %    IC{i}=10^(-6)*[I_table(5,i), -I_table(8,i), -I_table(9,i);
        %                  -I_table(8,i),  I_table(6,i), -I_table(10,i);
        %                  -I_table(9,i), -I_table(10,i), I_table(7,i)];
        %end
%......
%基座信息
% 基座角速度为0
omiga_v0 = [0; 0; 0];
% 基座角加速度为0
omiga_a0 = [0; 0; 0];
%重力因素
dv_0=[0 0 g]';%重力等效成向上为g的加速度
%迭代初始化
temp=1;%步长
tend=0; %结束时间
t_cont=0;%计数
%循环开始
% for t=0:temp:tend
t=3;
    t_cont=t_cont+1;
    %.......五次傅里叶级数
%     q0=sym('q0%d',[1,7]);
%     a=sym('a%d',[7,5]);
%     b=sym('b%d',[7,5]);
%      theta=sym(zeros(1,7));
%      d_theta=sym(zeros(1,7));
%     dd_theta=sym(zeros(1,7));
 [theta,d_theta,dd_theta]=position(t);
%     syms j
%     for i=1:7
%         for j=1:5
%             theta(i)=5*a(i,j)/(pi*j)*sind(180/5*j*t)-5*b(i,j)/(pi*j)*cosd(180/5*j*t)+theta(i);
%             d_theta(i)=a(i,j)*cosd(180/5*j*t)+b(i,j)*sind(180/5*j*t)+d_theta(i);
%             dd_theta(i)=-pi*j*a(i,j)/5*sind(180/5*j*t)+pi*j*b(i,j)/5*cosd(180/5*j*t)+dd_theta(i);
%         end
%         theta(i)=theta(i)+q(i);
%     end
     %前向差分求运动学参数
%     theta = [0 0 0 0 0 0 -pi/2*cos(pi*t)];%设置theta相关函数,规划运动轨迹
      %保存每次迭代的theta值
    for i=1:Number
        theta_cont(i,t_cont) = theta(i);
    end
    % 获取杆件间的旋转矩阵(i+1在i下的表示) i上 i+1下
    R=cell(1,Number);
    R0=cell(1,Number);    
    for i=1:Number
    R0{i} = [1,            0,             0;
             0,cosd(alpha(i)),-sind(alpha(i));
             0,sind(alpha(i)), cosd(alpha(i))];
    R{i} = [cos(theta(i)),  -sin(theta(i)), 0.0; 
            sin(theta(i)),  cos(theta(i)),  0.0; 
            0.0,            0.0,            1.0];
    R{i} = R0{i}*R{i};
    end

    %......增加连杆后在此增加连杆旋转矩阵
    %......
    %牛顿-欧拉方程线性化
    Z=[0 0 1]';%关节坐标旋转轴方向向量
    %外推
    %连杆1
	    omiga_v(:,1)=R{1}\omiga_v0+d_theta(1)*Z;
	    omiga_a(:,1)=R{1}\omiga_a0+cross(R{1}\omiga_v0, d_theta(1)*Z) +dd_theta(1)*Z;
	    dv(:,1)=R{1}\( cross(omiga_a0, P(:,1) )+cross( omiga_v0, cross( omiga_v0, P(:,1) ) )+dv_0 );
    %连杆2~Number
    for i=2:Number
	    omiga_v(:,i)=R{i}\omiga_v(:,i-1)+d_theta(i)*Z;
	    omiga_a(:,i)=R{i}\omiga_a(:,i-1)+cross( R{i}\omiga_v(:,i-1), d_theta(i)*Z ) +dd_theta(i)*Z;
	    dv(:,i)=R{i}\( cross( omiga_a(:,i-1), P(:,i) )+cross( omiga_v(:,i-1), cross(omiga_v(:,i-1),P(:,i) ) )+dv(:,i-1) );
    end
    %计算H矩阵
    H=cell(1,Number);
    for i=1:Number
        H{i}=inertia_force(omiga_v(1,i),omiga_v(2,i),omiga_v(3,i),omiga_a(1,i),omiga_a(2,i),omiga_a(3,i),dv(1,i),dv(2,i),dv(3,i));
    end
    %....
    %计算A矩阵
    A=cell(1,Number);
    for i=1:Number
        A{i}=Torque_A(omiga_v(1,i),omiga_v(2,i),omiga_v(3,i),omiga_a(1,i),omiga_a(2,i),omiga_a(3,i),dv(1,i),dv(2,i),dv(3,i));
    end
    %....
    %内推求Yf Y矩阵
    Yf=cell(1,Number);
    Y=cell(1,Number);
    for i=1:Number
        Yf{i}=zeros(3,70);
        Y{i}=zeros(3,70);
%         Y{i}=sym('A%d',[3,70]);
    end
    Yf{Number}(:,61:70)=H{Number};
    Y{Number}(:,61:70)=A{Number};
    for i=1:Number-1
        Yf{Number-i}(:,10*(Number-i)-9:10*(Number-i))=H{Number-i};
        Yf{Number-i}=Yf{Number-i}+R{Number-i}*Yf{Number-i+1};
        Y{Number-i}(:,10*(Number-i)-9:10*(Number-i))=A{Number-i};
        Pf=Cross_S(P(1,Number-i+1),P(2,Number-i+1),P(3,Number-i+1));
        Y{Number-i}=Y{Number-i}+R{Number-i}*Y{Number-i+1}+Pf*R{Number-i}*Yf{Number-i+1};
    end
    for i=1:Number
        L(i,:)=Y{i}(3,:);
    end
%{
    time(t_cont)=t;%计时
    for i=1:Number
    	tao(t_cont,i)=n(:,i)'*Z;
    end
%}
    d(1:2)=[0 0];d(4)=0;d(6:7)=[0 0];
d(3)=0.4495;d(5)=0.4495;
alpha=[0,90,-90,90,-90,90,-90];
a(1:3)=[0 0 0];a(6:7)= [0 0];
a(4)=0.03;a(5)=-0.03;
    P=Parameter_P(d,a,alpha);
tao_p=L*P;
% end