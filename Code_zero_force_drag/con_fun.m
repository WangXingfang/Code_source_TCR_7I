function [c,ceq] = con_fun(x)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
% for j=1:7
%     c(j)=0;
%     for i=1:5
%         c(j)=5/(pi*i)*sqrt(x(7*i+j)^2+x(7*i+35+j)^2)+c(j);
%     end
%         c(j)=c(j)+x(j);
% end

for j=1:7
    c(j)=0;
    for i=1:5
        c(j)=sqrt(x(7*i+j)^2+x(7*i+35+j)^2)+c(j);
    end
        c(j)=c(j)+x(j);
end
c(1)=c(1)-17/18*pi;
c(2)=c(2)-2/3*pi;
c(3)=c(3)-17/18*pi;
c(4)=c(4)-2/3*pi;
c(5)=c(5)-17/18*pi;
c(6)=c(6)-25/36*pi;
c(7)=c(7)-17/18*pi;
for j=8:14
    c(j)=0;
    for i=1:5
        c(j)=pi/5*i*sqrt(x(7*i+j-7)^2+x(7*i+28+j)^2)+c(j);
    end
end
c(8)=c(1)-104/180*pi;
c(9)=c(2)-113/180*pi;
c(10)=c(3)-113/180*pi;
c(11)=c(4)-113/180*pi;
c(12)=c(5)-209/180*pi;
c(13)=c(6)-140/180*pi;
c(14)=c(7)-140/180*pi;
for j=15:21
     c(j)=0;
    for i=1:5
        c(j)=(pi*i)^2/25*sqrt(x(7*i+j-14)^2+x(7*i+21+j)^2)+c(j);
    end
end
c(15)=c(15)-10;
c(16)=c(16)-10;
c(17)=c(17)-10;
c(18)=c(18)-10;
c(19)=c(19)-10;
c(20)=c(20)-10;
c(21)=c(21)-10;
ceq(1)=0;
ceq(2)=0;
for i=8:43
    ceq(1)=x(i)+ceq(1);
end
for i=1:7
   ceq(2)=x(i+42)+2*x(i+49)+3*x(i+56)+4*x(i+63)+5*(i+70)+ceq(2);
end
end

