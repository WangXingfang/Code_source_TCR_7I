clear
N=1;
for n=1:4
    i=0
    N=2*N;
    for t=0:0.01:2
        i=i+1;
        y(i,1)=1/(1+t^(2*N));
    end
    t=0:0.01:2
    plot(t,y(:,1),'LineWidth',2);
    set(gca,'FontSize',16,'FontName','Times New Roman');
    hold on
end
grid on
xlabel('w/wc')
ylabel('|H(jw)|^2')
axis([0,2,0,1.2]);
legend('N=2','N=4','N=8','N=16')
% plot(t,Tx_average,'LineWidth',2,'color',[163,73,164]/255);
% set(gca,'xlim',[0,(t_max - t_min)/2]);
% set(gca,'FontSize',16,'FontName','Times New Roman');
% grid on;
