load A1
[num,den]=size(A1);
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
    tao(:,i)=filtfilt(b,a,A1(:,i)); %滤波b、a滤波器系数，my滤波前序列
end
for i=1:7
    figure(i);
    plot(t,A1(:,i));
    figure(i+7);
    plot(t,mmyy(:,i));
    hold on
    plot(t,A1(:,i));
    figure(i+14)
    plot(t,tao(:,i));
end