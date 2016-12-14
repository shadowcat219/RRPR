clc
clear

% % %% For Testing All the functions
% syms g the1 the2 L1 L2 m1 m2 
% R=[cos(the1) -sin(the1) 0 cos(the2) -sin(the2) 0;
%     sin(the1) cos(the1) 0  sin(the2) cos(the2) 0; 
%     0 0 1 0 0 1]
% P=[0 L1 L2;0 0 0;0 0 0]
% acc0_0=[0;g;0]
% Pc=[L1 L2;0 0;0 0]
% m=[m1 m2];
% Ic=[zeros(3) zeros(3)];
% dVc=sym(zeros(3,4));
% F=sym(zeros(3,4));
% N=sym(zeros(3,4));

% %% For Testing a prismatic joint
% syms g the1 the2 L1 L2 m1 m2  Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 
% 
% R=[1 0 0 cos(the2) -sin(the2) 0;
%     0 1 0  sin(the2) cos(the2) 0; 
%     0 0 1 0 0 1]
% P=[0 0 0;0 0 L2;0 L1 0]
% acc0_0=[g;0;0]
% Pc=[0 0;0 0;-L1 0]
% m=[m1 m2];
% Ic=[Ixx1 0 0  Ixx2 0 0 0 0 0; 
%     0 Iyy1 0 0 Iyy2 0 0 0 0; 
%     0 0 Izz1 0 0 Izz2 0 0 0];

% This code calcuates the formulas for the dynamic simulations
% Use 'demo1.m' to get rotation and transformation matices
demo1 %#codegen
% Find appropriate C++ equivalent
% export data
%R=[R0_1, R1_2, R2_3, R3_4 T4_5(1:3,1:3)]
R=[T0_1(1:3,1:3), T1_2(1:3,1:3), T2_3(1:3,1:3), T3_4(1:3,1:3),T4_5(1:3,1:3)]
P=[T0_1(1:3,4), T1_2(1:3,4), T2_3(1:3,4), T3_4(1:3,4),T4_5(1:3,4)]

syms g

m=[1.7,1.0, 1.7, 1.0]; % kg

% Check with group and Professor
Pc1_1=[0;0;0];
Pc2_2=[0;0;0];
Pc3_3=[0;0;0];
Pc4_4=[30;0;0] % According to our DH paramaters the COM is 30mm along {4} X-axis
Pc=[Pc1_1,Pc2_2,Pc3_3,Pc4_4]

%  apply a fictional upward acc to link 0 
acc0_0=[0;0;g];

%  Only one Ic is non-zero
Ic4=m(4)*(transpose(Pc4_4)*Pc4_4*eye(3)-Pc4_4*transpose(Pc4_4));
Ic=[zeros(3) zeros(3) zeros(3) Ic4];

% create an empty matrix NOT VALID in C++
dVc=sym(zeros(3,4));
F=sym(zeros(3,4));
N=sym(zeros(3,4));

%%  Run the dynamic equations for every joint
di_1=sym('dj_%d',[1 4]);
ddi_1=sym('ddj_%d',[1 4]);
%tau=sym('tau_%d',[1 4])

for i=1:4 
    if(i==1) %(i==0)
        joint=0; %1
        %joint=1; %For testing
        omegai=[0;0;0];
        d_omegai=[0;0;0];
        dVi=acc0_0;
        cI=Ic(:,3*(i)-2:3*(i));
        
    elseif(i==3) % else if(i==2)
        joint=1;
        omegai=omegai_1;
        d_omegai=d_omegai_1;
        dVi=dVi_1;
        cI=Ic(:,3*(i)-2:3*(i));
    else
        joint=0;
        omegai=omegai_1;
        d_omegai=d_omegai_1;
        dVi=dVi_1;
        cI=Ic(:,3*(i)-2:3*(i));
    end
    disp('*************************')
    [omegai_1, d_omegai_1, dVi_1, dVci_1, Fi_1, Ni_1] = dynamicsEQN(joint,omegai,d_omegai,R(:,3*i-2:3*i),P(:,i),dVi,Pc(:,i),m(i),di_1(i),ddi_1(i),cI);
   
   
    %  Move results into main matrix
    dVc(:,i)=vpa(dVci_1(:));
    F(:,i)=vpa(Fi_1(:));
    N(:,i)=vpa(Ni_1(:));
    
end
%%

i=4
%i=2; %FOR TESTING
torque=sym(zeros(3,1));
while i>0 
    if (i==4) % free movement
        fi_1=[0;0;0];
        ni_1=[0;0;0];
        joint=0%1;
        r=zeros(3);
   elseif(i==3) % else if(i==2)
        joint=1;
        fi_1=fi;
        ni_1=ni;
       r= R(:,3*(i+1)-2:3*(i+1));
    else
        fi_1=fi
        ni_1=ni
        joint=0;
        r=R(:,3*(i+1)-2:3*(i+1));
        %joint =1; %testing
    end 
    i
    disp('*************************')
    [fi, ni, torquei]=newtonEulerEQN(joint,r,fi_1, F(:,i),ni_1,N(:,i),Pc(:,i),P(:,i+1))

    torque(i)=simplify(torquei);
    i=i-1;
end
%%
syms tau1 tau2 tau3 tau4 ddj_1 ddj_2 ddj_3 ddj_4 dj_1 dj_2 dj_3 dj_4
 disp('*************************')
[M,B] = equationsToMatrix([torque(1)==tau1 torque(2)==tau2 torque(3)==tau3 torque(4)==tau4],[ddj_1 ddj_2 ddj_3 ddj_4] );
M=simplify(M)


[G,bb ]=equationsToMatrix([B(1)-tau1==tau1 B(2)-tau2==tau2 B(3)-tau3==tau3 B(4)-tau4==tau4],g);
V=simplify(bb-[tau1;tau2; tau3;tau4])
G=simplify(G)
%%
Minv=inv(M)