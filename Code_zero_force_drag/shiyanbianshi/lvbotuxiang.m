load A2
A=A2;
tao = lvbo_tao(A);
[num,den]=size(A);
t=0:0.001:(num-1)/1000;
for i=1:7
    figure(i);
    t=0:0.001:(num-1)/1000;
    plot(0.973*t,-A(1:1:10301,i),'LineWidth',0.5,'color',[0,0,255]/255);
    set(gca,'xlim',[0,10]);
%     set(gca,'FontSize',16,'FontName','Times New Roman');
    hold on
    t=0:0.001:(num-1)/1000;
    plot(0.973*t,-tao(1:1:10301,i),'LineWidth',2,'color',[255,0,0]/255);
    set(gca,'xlim',[0,10]);
    xlabel('时间（s）')
    ylabel('力矩（N·m）')
 %    set(gca,'FontSize',16,'FontName','Times New Roman');
 legend('原始力矩','滤波后力矩')
 title(['关节',num2str(i)]);
 grid on
end