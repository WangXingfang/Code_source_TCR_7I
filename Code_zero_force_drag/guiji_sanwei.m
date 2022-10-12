temp=0.01;
tend=10;
x0=[0;0;100;899];
t_cont=1;
for t=0:temp:tend
	[q,dq,ddq]=position(t);
    T=ForwardKinematics(q);
    x(:,t_cont)=T*x0;
    t_cont=t_cont+1;
end

plot3(x(1,:),x(2,:),x(3,:));
grid on;
% mesh(x(2,:),x(1,:),x(3,:));
% surf(x(2,:),x(1,:),x(3,:));