% FUNCTION INPUTS:
%  omegai == the angular velocity of link i
%  d_omegai == the angular acceleration of link i
%  i == previous link number
%  i_1 == i+1 which is the current link number
%  R == rotation matrix of Ri_(i+1)
%  P == position vector which locates the origin of {i+1} relative to {i}.
%  dVi == linear velocity of link {i}
%  Pc == location of COM for {i+1}
%  m= mass at COM
%  cIi_1 = Inertica matrix at the COM
%  di_1 =either the angular or linear velocity of joint
%  ddi_1 =either the angular or linear acceleration of joint

function [omegai_1, d_omegai_1, dVi_1, dVci_1, Fi_1, Ni_1] =dynamicsEQN(joint,omegai,d_omegai,R,P,dVi,Pci_1,m,di_1,ddi_1,cIi_1) 
R
if (joint==0) % Rotational
 % angular velocity 
    omegai_1=transpose(R)*omegai+[0;0;di_1]

% angular accel
   a=transpose(R)*d_omegai;
   c=[0;0;ddi_1];
   if(omegai~=0);
     b=cross(transpose(R)*omegai,[0;0;di_1]);
   else
     b=[0;0;0];
   end
   d_omegai_1=a+b+c

 % linear accel
        a=cross(d_omegai,P);
        b=cross(omegai,cross(omegai,P));
    dVi_1=simplify(transpose(R)*(a+b+dVi))
     
elseif (joint ==1) % Prismatic
% angular velocity
    omegai_1=transpose(R)*omegai
    
% angular accel
     d_omegai_1=transpose(R)*d_omegai

% linear accel
        a=cross(d_omegai,P);
        b=cross(omegai,cross(omegai,P))+dVi;
        c=cross(2*omegai_1,[0;0;di_1]);
    dVi_1=simplify(transpose(R)*(a+b)+c+[0;0;ddi_1])

end

% linear accel wrt to COM
    a=cross(d_omegai_1,Pci_1);
    b=cross(omegai_1,Pci_1);
    c=cross(omegai_1,b);
    dVci_1=simplify(a+c+dVi_1)


% Because of the point-mass assumption, the inertia tensor written at the center of
% mass for each link is the zero matrix:
    Fi_1=simplify(m*dVci_1)
%     cIi_1
%     if(cIi_1==0)
        Ni_1=[0;0;0]
%     else
%         d_omegai
%         Ni_1=cIi_1*d_omegai_1+cross(omegai_1,cIi_1*omegai_1)
%     end


end

