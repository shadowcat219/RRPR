%choose a via point that is possible
clc
clear
tv=10
the0=90
theV=95
thef=99

tf=20
RES=0.03
time=0:RES:tf;
time1=0:RES:tv;
time2=5:RES:tf;

%%
% Question a)
syms a b c d t e %f 
V=a+b*t+c*t^2+d*t^3+e*t^4%+f*t^5
Vp=diff(V,t)
Vpp=diff(V,t,2)

V0=subs(V, t,0)==the0
V1=subs(V, t,tv)==theV %theV
Vp0=subs(Vp,t,0)==0
Vp1=subs(Vp,t,tv)==vpa(((the0-theV)/(tv-0)+(thef-theV)/(tf-tv))/2)
Vpp0=subs(Vpp,t,0)==0
Vpp1=subs(Vpp,t,tv)==0

[A,B] = equationsToMatrix([V0,V1, Vp0,Vp1,Vpp0,Vpp1],[a b c d e] )%[a b c d e f] )
X = linsolve(A,B)
X=double(X)
X= flipud(X)
x=polyval(X,time1);

Xp=polyder(X);
xp=polyval(Xp,time1);

Xpp=polyder(Xp);
xpp=polyval(Xpp,time1);


[c,r]=size(X)

for i=1:r
    if(x(i)>360)
        x(i)=mod(x(i),360);
    end 
end
%%
syms a b c d t %e f
U=a+b*t+c*t^2+d*t^3%+e*t^4+f*t^5
Up=diff(U,t)
Upp=diff(U,t,2)

U0=subs(U,t,tv)==theV
U1=subs(U,t,tf)==thef
Up0=subs(Up,t,tv)==((the0-theV)/(tv-0)+(thef-theV)/(tf-tv))/2
Up1=subs(Up,t,tf)==0
Upp0=subs(Upp,t,tv)==0
Upp1=subs(Upp,t,tf)==0

[A,B] = equationsToMatrix([U0,U1, Up0,Up1,Upp0,Upp1],[a b c d ])%[a b c d e f] )
Y = linsolve(A,B)
Y=double(Y)
Y= flipud(Y)
y=polyval(Y,time2);

Yp=polyder(Y);
yp=polyval(Yp,time2);

Ypp=polyder(Yp);
ypp=polyval(Ypp,time2);

[c,r]=size(y)

for i=1:r
    if(y(i)>360)
        y(i)=mod(y(i),360);
    end 
end
%%
figure
    plot(time1,x) 
    hold on;
    plot(time2,y)
    hold off;
   title('Two Splines')
    ylabel('Joint')
    xlabel('time(sec)' )
    
figure % new figure
    plot(time1,xp) 
    hold on;
    plot(time2,yp)
     title('Two Splines')
    ylabel('Velocity ')
    xlabel('time(sec)' )
hold off;

figure % new figure
    plot(time1,xpp) 
    hold on;
    plot(time2,ypp)
    hold off;
    title('Two Splines')
    ylabel('Accelaration')
    xlabel('time(sec)')