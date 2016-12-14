syms the1 the2 thet3 the4
R0_1=rotationT(the1, 0)
R1_2=rotationT(the2, 0)
R2_3=rotationT(0, pi())
R3_4=rotationT(the4, 0)
R2_3(2,3)=0
R2_3(3,2)=0
R0_1*R1_2*R2_3*R3_4


%% Tmatirx(theta, alpha_1, a_1, d)
%
syms the1 the2 thet3 the4 d3 px py the12 the12_star
 T0_1=Tmatrix(the1, 0,0,405) 
% T0_1(2,1)=0;
% T0_1(2,2)=0;
% T0_1(3,3)=0;
% T0_1(3,4)=0;
T0_1=Tmatrix(the1, 0,0,405) 
T1_2=Tmatrix(the2, 0,195,070)

T2_3=Tmatrix(0, pi(),142,d3) %%-d3
T2_3(2,3)=0;
T2_3(3,2)=0;
T2_3(2,4)=0;
T2_3
T3_4=Tmatrix(the4, 0,0,(410+080))
T4_5=Tmatrix(0, 0,0,060)

T=T0_1*T1_2*T2_3*T3_4*T4_5
T=simplify(T)

%% Given the2 Solve for the1
%%thet2=60
% simplify(T(1,4))
% simplify(T(2,4))
% px-142*cos(the1 + the2)==195*cos(the1)
% py-142*sin(the1 + the2)==195*sin(the1)
% 
% (px-142*cos(the1 + the2))^2==(195*cos(the1))^2
% (py-142*sin(the1 + the2))^2==(195*sin(the1))^2
% 
% simplify(expand((px-142*cos(the1 + the2))^2+(py-142*sin(the1 + the2))^2))==2*(195)^2
% 
% simplify(expand((px-142*cos(the1 + the2))^2+(py-142*sin(the1 + the2))^2))...
% -px^2-py^2-20164==2*(195)^2+px^2+py^2+20164
% 
% (simplify(expand((px-142*cos(the1 + the2))^2+(py-142*sin(the1 + the2))^2))...
% -px^2-py^2-20164)/(-284)==(2*(195)^2+px^2+py^2+20164)/(-284)
% 
% disp('same as acos(theta)+bsin(theta)=c')
% a=px
% b=py
% c=(2*(195)^2+px^2+py^2+20164)/(-284)
% 
% the12==2*atan2((b+sqrt(b^2-c^2+a^2))/(c+a),1)
% the12_star==2*atan2((b-sqrt(b^2-c^2+a^2))/(c+a),1)
