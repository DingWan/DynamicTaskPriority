function d=qiudiandaozhixianjuli(P,Q,R)
syms t;
direct_v=[P(1)-Q(1),P(2)-Q(2),P(3)-Q(3)];
if direct_v(1)==0 
    if direct_v(2)==0
            x0=P(1);
            y0=P(2);
            z0=t;
    else
         x0=P(1);
         y0=t;
         z0=(t-P(2))*direct_v(3)/direct_v(2)+P(3);
    end
else
        x0=t;
        y0=(t-P(1))*direct_v(2)/direct_v(1)+P(2);
        z0=(t-P(1))*direct_v(3)/direct_v(1)+P(3);
end
%chuixianxiangliang
m=[x0-R(1) y0-R(2) z0-R(3)];
f=m(1)*direct_v(1)+m(2)*direct_v(2)+m(3)*direct_v(3);
%t=solve('m(1)*direct_v(1)+m(2)*direct_v(2)+m(3)*direct_v(3)=0','t')
t=solve(f,'t');
if direct_v(1)==0 
    if direct_v(2)==0
            x0=P(1);
            y0=P(2);
            z0=t;
    else
         x0=P(1);
         y0=t;
         z0=(t-P(2))*direct_v(3)/direct_v(2)+P(3);
    end
else
        x0=t;
        y0=(t-P(1))*direct_v(2)/direct_v(1)+P(2);
        z0=(t-P(1))*direct_v(3)/direct_v(1)+P(3);
end
d=sqrt((x0-R(1))^2+(y0-R(2))^2+(z0-R(3))^2);