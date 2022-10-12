function [c,ceq] = con_guijijili(x)
q0=x(1:7);
for i=1:5
    a(1:7,i)=x(7*i+1:7*i+7);
end
for i=1:5
    b(1:7,i)=x(7*i+36:7*i+42);
end
for j=1:7
    c(j)=0;
    for i=1:5
        c(j)=5/(pi*i)*sqrt(a(j,i)^2+b(j,i)^2)+c(j);
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
c(22)=c(1)-2*x(1)-17/18*pi;
c(23)=c(2)-2*x(2)-2/3*pi;
c(24)=c(3)-2*x(3)-17/18*pi;
c(25)=c(4)-2*x(4)-2/3*pi;
c(26)=c(5)-2*x(5)-17/18*pi;
c(27)=c(6)-2*x(6)-25/36*pi;
c(28)=c(7)-2*x(7)-17/18*pi;
for j=8:14
    c(j)=0;
    for i=1:5
        c(j)=sqrt(a(j-7,i)^2+b(j-7,i)^2)+c(j);
    end
end
c(8)=c(1)-104/180*pi;
c(9)=c(2)-104/180*pi;
c(10)=c(3)-113/180*pi;
c(11)=c(4)-113/180*pi;
c(12)=c(5)-209/180*pi;
c(13)=c(6)-140/180*pi;
c(14)=c(7)-140/180*pi;
for j=15:21
     c(j)=0;
    for i=1:5
        c(j)=pi/5*i*sqrt(a(j-14,i)^2+b(j-14,i)^2)+c(j);
    end
end
c(15)=c(15)-10;
c(16)=c(16)-10;
c(17)=c(17)-10;
c(18)=c(18)-10;
c(19)=c(19)-10;
c(20)=c(20)-10;
c(21)=c(21)-10;
for i=1:7
    ceq(i)=0;
    for j=1:5
        ceq(i)=ceq(i)+a(i,j);
    end
end
for i=8:14
    ceq(i)=0;
    for j=1:5
        ceq(i)=ceq(i)+j*pi/5*b(i-7,j); 
    end
end
end

