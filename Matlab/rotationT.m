function R=rotationT(theta, alpha_1)

R=[cos(theta) -sin(theta) 0;
sin(theta)*cos(alpha_1) cos(theta)*cos(alpha_1) -sin(alpha_1);
sin(theta)*sin(alpha_1) cos(theta)*sin(alpha_1) cos(alpha_1);];

end

%%
% syms the1 the2 thet3 the4
% R1_0=rotationT(the1, 0)
%  R2_1=rotationT(the2, 0)
%   R3_2=rotationT(0, pi())
%   R4_3=rotationT(the4, 0)
%   R3_2(2,3)=0
%   R3_2(3,2)=0
%   R1_0*R2_1*R3_2*R4_3