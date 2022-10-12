load A1
q=A1(:,8:14);
[num,den]=size(A1);
temp=0.001;
for t=2:num
    dq(t,:)=(q(t,:)-q(t-1,:))/temp;
end
dq(1,:)=dq(2,:);
for t=2:num
    ddq(t,:)=(dq(t,:)-dq(t-1,:))/temp;
end
ddq(1,:)=ddq(2,:);
t=0:0.001:(num-1)/1000;
fn=1000;%采样频率
ap=1;%通带最大衰减
as=50;%阻带最大衰减

wp=1;%通带截止频率
ws=10; %阻带截止频率

wpp=wp/(fn/2);
wss=ws/(fn/2); %归一化;
[n wn]=buttord(wpp,wss,ap,as) ;%计算阶数截止频率
[b a]=butter(n,wn);%计算N阶巴特沃斯数字滤波器系统函数分子、分母多项式的系数向量b、a。
for i=1:7
    qq(:,i)=filtfilt(b,a,q(:,i));
    dqq(:,i)=filtfilt(b,a,dq(:,i));
    ddqq(:,i)=filtfilt(b,a,ddq(:,i));
end
t=0:0.001:(num-1)/1000;
for i=1:7
    figure(i);
    plot(t,ddqq(:,i));
end
