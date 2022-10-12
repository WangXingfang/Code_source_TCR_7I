load A3
A=A3;
fk=1000 ;%采样频率
N=10301; %采样个数
n=-N/2:N/2-1;
f=n*fk/N;
t=n/fk;
y=A(:,7);
fft_y=fft(y);
fftshift_y=fftshift(fft_y);
f=linspace(0,25,5151);
plot(f,abs(fftshift_y(5151:10301)),'LineWidth',0.5);%滤波前频谱分析
xlabel('频率(Hz)')
ylabel('幅值')
set(gca,'xlim',[0,25]);
% set(gca,'FontSize',16,'FontName','Times New Roman');
% grid on

% for i=1:3
%     subplot(2,12,[2*i,2*i+1])
% y=A3(:,i);
% fft_y=fft(y);
% fftshift_y=fftshift(fft_y);
% f=linspace(-25,25,10301);
% plot(f,abs(fftshift_y));%滤波前频谱分析
% xlabel('频率(Hz)')
% ylabel('幅值')
% end
% for i=4:7
%     subplot(2,8,[2*i+1,2*i+2])
% y=A3(:,i);
% fft_y=fft(y);
% fftshift_y=fftshift(fft_y);
% f=linspace(-25,25,10301);
% plot(f,abs(fftshift_y));%滤波前频谱分析
% xlabel('频率(Hz)')
% ylabel('幅值')
% end
% axis([0,25,0,1.2]);
% [g,Wn]=buttord(0.002,0.02,1,50)
% [b,a]=butter(g,Wn);
%  [q,w]=freqz(b,a,256);
%  fs=30;
%  plot(w*fs/(2*pi),abs(q))
%  k=filter(b,a,y);
%  fft_k=fft(k);
%  fftshift_k=fftshift(fft_k);
%  plot(f,abs(fftshift_k));