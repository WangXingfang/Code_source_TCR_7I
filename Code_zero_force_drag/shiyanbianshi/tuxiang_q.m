clear all
load A2
load x2
x=x2;
A=A2;
[qq,dqq,ddqq] = lvbo_q(A);
[num,den]=size(A);
t=0:0.001:(num-1)/1000;
q0=x(1:7);
for i=1:5
    a(:,i)=x(7*i+1:7*i+7);
end
for i=1:5
    b(:,i)=x(7*i+36:7*i+42);
end
%1
% q0(2)=q0(2)+1;
%2...
 q0(7)=q0(7)+1;
q0(2)=2/3*q0(2)-0.3;
a(2,:)=2/3*a(2,:);
b(2,:)=2/3*b(2,:);
%3...
% q0(7)=q0(7)+1.4;%3
% q0=roundn(q0,-3);
% a=roundn(a,-3);
% b=roundn(b,-3);
q=zeros(1,7);
dq=zeros(1,7);
ddq=zeros(1,7);
syms j
% temp=0.1;
tend=10;
t_cont=0;
x_t0=[0;0;100;899];
for t=0:0.01:tend
    for i=1:7
        q(i)=0;
        dq(i)=0;
        ddq(i)=0;
	    for j=1:5
    		q(i)=a(i,j)*sin(pi/5*j*t)+b(i,j)*cos(pi/5*j*t)+q(i);
    		dq(i)=a(i,j)*pi/5*j*cos(pi/5*j*t)-b(i,j)*pi/5*j*sin(pi/5*j*t)+dq(i);
    		ddq(i)=-pi^2*j^2*a(i,j)/25*sin(pi/5*j*t)-pi^2*j^2*b(i,j)/25*cos(pi/5*j*t)+ddq(i);
	    end
	    q(i)=q(i)+q0(i);
    end
    T=ForwardKinematics(q);
    xt(:,t_cont+1)=T*x_t0*0.001;
    t_cont=t_cont+1;
    for i=1:7
        q_jishu(t_cont,i)=q(i);
         q_jishu=roundn(q_jishu,-5);
    end
    for i=1:7
        y(t_cont,i)=q(i);
        dy(t_cont,i)=dq(i);
        ddy(t_cont,i)=ddq(i);
    end
    m(t_cont)=t;
end
%激励轨迹理想值图像
% for i=1:7
%      figure(i);
% %      title(['关节',num2str(i)]);
%      subplot(3,1,1);
% plot(m,y(:,i),'LineWidth',2);
% xlabel('时间(s)');
% ylabel('')
% grid on;
% subplot(3,1,2);
% plot(m,dy(:,i),'LineWidth',2);
% xlabel('时间(s)');角度(rad)
% ylabel('角速度(rad/s)');
% grid on;
% subplot(3,1,3);
% plot(m,ddy(:,i),'LineWidth',2);
% xlabel('时间(s)');
% ylabel('角加速度(rad/s^2)');
% grid on;
% %     plot(m,y(:,i));
% %     xlabel('时间(s)');
% %     ylabel('角度(rad)')
% %     title(['关节',num2str(i)]);
% %     hold on
% end
% figure(8);
plot3(xt(1,:),xt(2,:),xt(3,:),'LineWidth',2);
xlabel('x(mm)');
ylabel('y(mm)');
zlabel('z(mm)');
grid on;
save x.txt -ascii xt
%....

% t=0:0.001:(num-1)/1000;
%位置图
% for i=1:7
%      figure(i)
%      plot(m,y(:,i),'LineWidth',2,'color',[0,0,255]/255);
%     hold on
%     plot(0.973*t,qq(:,i),'LineWidth',2,'color',[255,0,0]/255);
%     set(gca,'xlim',[0,10]);
%     xlabel('时间(s)')
%     ylabel('角度(rad)')
%     title(['关节',num2str(i)]);
%     legend('理论位置','采集位置')
%     grid on
% end
% plot(0.973*t,qq(:,:),'LineWidth',2);
%     set(gca,'xlim',[0,10]);
%     xlabel('时间(s)')
%     ylabel('角度(rad)')
%     legend('1','2','3','4','5','6','7')
%     grid on

%     i=7;
%     figure(1)
%     plot(m,y(:,i),'LineWidth',2,'color',[0,0,255]/255);
%     hold on
%     plot(0.973*t,qq(:,i),'LineWidth',2,'color',[255,0,0]/255);
%     set(gca,'xlim',[0,10]);
%     xlabel('时间(s)')
%     ylabel('角度(rad)')
%     title(['关节',num2str(i)]);
%     legend('理论位置','采集位置')
%     grid on
%      figure(2)
%      plot(m,y(:,i),'LineWidth',4,'color',[0,0,255]/255);
%     hold on
%     plot(0.973*t,qq(:,i),'LineWidth',4,'color',[255,0,0]/255);
%     set(gca,'xlim',[0,10]);
%     set(gca,'FontSize',30,'FontName','Times New Roman');
%     grid on

%角速度图
% for i=1:7
%      figure(i)
%      plot(m,dy(:,i),'LineWidth',2,'color',[0,0,255]/255);
%     hold on
%     plot(0.973*t,dqq(:,i),'LineWidth',2,'color',[255,0,0]/255);
%     set(gca,'xlim',[0,10]);
%     xlabel('时间(s)')
%     ylabel('角速度(rad)')
%     title(['关节',num2str(i)]);
%     legend('理论角速度','差分角速度')
%     grid on
% end
%角加速度
% for i=1:7
%      figure(i)
%      plot(m,ddy(:,i),'LineWidth',2,'color',[0,0,255]/255);
%     hold on
%     plot(0.973*t,ddqq(:,i),'LineWidth',2,'color',[255,0,0]/255);
%     set(gca,'xlim',[0,10]);
%     xlabel('时间(s)')
%     ylabel('角加速度(rad)')
%     title(['关节',num2str(i)]);
%     legend('理论角加速度','差分角加速度')
%     grid on
% end
% for i=1:7
%      figure(i)
%      plot(m,dy(:,i),'LineWidth',2,'color',[0,0,255]/255);
%     hold on
%     plot(0.973*t,dqq(:,i),'LineWidth',2,'color',[255,0,0]/255);
%     set(gca,'xlim',[0,10]);
%     xlabel('时间(s)')
%     ylabel('角速度(rad/s)')
%     title(['关节',num2str(i)]);
%     grid on
% end
% for i=1:7
%      figure(i)
%      plot(m,ddy(:,i),'LineWidth',2,'color',[0,0,255]/255);
%     hold on
%     plot(0.973*t,ddqq(:,i),'LineWidth',2,'color',[255,0,0]/255);
%     set(gca,'xlim',[0,10]);
%     xlabel('时间(s)')
%     ylabel('角加速度(rad/s^2)')
%     title(['关节',num2str(i)]);
%     grid on
% end
% for i=1:7
%     figure(i);
% %     plot(t,q(:,i));
% %     hold on
%     plot(t,qq(:,i));
% end