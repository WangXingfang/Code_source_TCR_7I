clear ; clc; close all;
load I_table
g=9.80665; %�������ٶ�: m/s^2
%�ؽ���Ϣ
Number=7;%������
%L=[0.4,0.4]; %�˳�/m
% i+1����ϵԭ����i����ϵ�µı�ʾ
P = [  0,   0,     0; 
	   0,   0,     0;
       0,   0.4495, 0;
       0.03,  0,     0;
      -0.03,  0.4495, 0;
       0,   0,     0;
       0,   0,     0;]';
%d=[0.2,0.2]; %���ľ�ؽ�λ��
% �����ڹؽ�����ϵ�µı�ʾ, unit: m
PC = 10^(-3)*I_table(2:4,:);
%alpha����,unit:rad
alpha=[0,pi/2,-pi/2,pi/2,-pi/2,pi/2,-pi/2];
m=I_table(1,:); %����, unit: kg
% �˼��ڹؽ�����ϵ�µĹ������� unit: kg*m^2
%......���������Ӻ����IC�ڴ棬����ת����������
IC=cell(1,Number);
for i=1:Number
    IC{i}=10^(-6)*[I_table(5,i), -I_table(8,i), -I_table(9,i);
                  -I_table(8,i),  I_table(6,i), -I_table(10,i);
                  -I_table(9,i), -I_table(10,i), I_table(7,i)];
end
%......
%������Ϣ
% �������ٶ�Ϊ0
omiga_v0 = [0; 0; 0];
% �����Ǽ��ٶ�Ϊ0
omiga_a0 = [0; 0; 0];
%��������
dv_0=[0 0 g]';%������Ч������Ϊg�ļ��ٶ�
%������ʼ��
temp=0.01;%����
tend=2; %����ʱ��
t_cont=0;%����
%ѭ����ʼ
for t=0:temp:tend
    t_cont=t_cont+1;
    %ֱ���󵼼����˶�ѧ����
%    theta=[-pi/2*cos(pi*t) -pi/2*cos(pi*t) 0 0 0 0 0];
%    for i=1:7
%        d_theta(i)=diff(theta(i));
%        dd_theta(i)=diff(d_theta(i));
%    end

%    theta = [0,-pi/2*cos(pi*t), 0, -pi/2*cos(pi*t), 0, -pi/2*cos(pi*t), 0];
%    d_theta= [0,pi^2/2*sin(pi*t), 0, pi^2/2*sin(pi*t), 0,  pi^2/2*sin(pi*t),0];
%    dd_theta=[0,pi^3/2*cos(pi*t), 0, pi^3/2*cos(pi*t), 0,  pi^3/2*cos(pi*t),0];
   theta = [0,0, 0, 0, 0, 0, 0];
   d_theta= [0,0, 0, 0, 0, 0,0];
   dd_theta=[0,0, 0, 0, 0, 0,0];
     %ǰ�������˶�ѧ����
%     theta = [0 0 0 0 0 0 -pi/2*cos(pi*t)];%����theta��غ���,�滮�˶��켣
      %����ÿ�ε�����thetaֵ
    for i=1:Number
        theta_cont(i,t_cont) = theta(i);
    end
%     if t==0
%         %��ʼ�ؽڽ�
%         %���ݳ�ʼʱ��״̬ȷ������ֵ
%        d_theta = [-pi/2, -pi/2,0,0,0,0,0];
%        dd_theta = [0, 0,0,0,0,0,0]; 
%     else if t==temp
%             d_theta = ( theta-theta_cont(:,t_cont-1) )/temp;%ǰ������d_theta
%             dd_theta = [0, 0,0,0,0,0,0];
%         else        
%         d_theta = ( theta-theta_cont(:,t_cont-1) )/temp;%��ǰ�����d_theta
%         dd_theta = ( theta-2*theta_cont(:,t_cont-1)+theta_cont(:,t_cont-2) )/temp^2;%ǰ������dd_theta
%         end
%     end

    % ��ȡ�˼������ת����(i+1��i�µı�ʾ) i�� i+1��
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

    %......�������˺��ڴ�����������ת����
    %......
    %ţ��-ŷ������
    Z=[0 0 1]';%�ؽ�������ת�᷽������
    %����
    %����1
	    omiga_v(:,1)=R{1}\omiga_v0+d_theta(1)*Z;
	    omiga_a(:,1)=R{1}\omiga_a0+cross(R{1}\omiga_v0, d_theta(1)*Z) +dd_theta(1)*Z;
	    dv(:,1)=R{1}\( cross(omiga_a0, P(:,1) )+cross( omiga_v0, cross( omiga_v0, P(:,1) ) )+dv_0 );
	    dv_c(:,1)=cross( omiga_a(:,1),PC(:,1) )+cross( omiga_v(:,1),cross( omiga_v(:,1),PC(:,1) ) )+dv(:,1);
	    F(:,1)=dv_c(:,1)*m(1);
	     N(:,1)=IC{1}*omiga_a(:,1)+cross(omiga_v(:,1),IC{1}*omiga_v(:,1));
    %����2~Number
    for i=2:Number
	    omiga_v(:,i)=R{i}\omiga_v(:,i-1)+d_theta(i)*Z;
	    omiga_a(:,i)=R{i}\omiga_a(:,i-1)+cross( R{i}\omiga_v(:,i-1), d_theta(i)*Z ) +dd_theta(i)*Z;
	    dv(:,i)=R{i}\( cross( omiga_a(:,i-1), P(:,i) )+cross( omiga_v(:,i-1), cross(omiga_v(:,i-1),P(:,i) ) )+dv(:,i-1) );
	    dv_c(:,i)=cross( omiga_a(:,i),PC(:,i) )+cross( omiga_v(:,i),cross( omiga_v(:,i),PC(:,i) ) )+dv(:,i);
	    F(:,i)=dv_c(:,i)*m(i);
	    N(:,i)=IC{i}*omiga_a(:,i)+cross( omiga_v(:,i),IC{i}*omiga_v(:,i) );
    end
    %����
    %����Number
	    f(1:3,Number)=F(:,Number);
	    n(1:3,Number)=N(:,Number)+cross(PC(:,Number), F(:,Number));
    %����1~Number-1
    for i=1:Number-1
	    f(1:3,Number-i)=R{Number-i+1}*f(1:3,Number-i+1)+F(:,Number-i);
	    n(1:3,Number-i)=N(:,Number-i)+R{Number-i+1}*n(1:3,Number-i+1)+cross( PC(:,Number-i), F(:,Number-i) )+cross( P(:,Number-i+1), R{Number-i+1}*f(1:3,Number-i+1) );
    end
    time(t_cont)=t;%��ʱ
    for i=1:Number
    	tao(t_cont,i)=n(:,i)'*Z;
    end
end

%Ϊ������ؽ�����x.t�ļ���������
Torquechart(:,1)=time(1,:)';%ʱ��
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