
load A1
for i=1:7
    figure(i)
    t=0:0.001:10.3;
%     plot(0.973*t,A1(:,i+7));
    plot(0.973*t,qq(:,i));
    t=0:0.01:10;
    hold on
    plot(t,y(:,i));
end