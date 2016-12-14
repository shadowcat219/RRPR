% FUNCTION INPUTS:
% R == rotation matrix of Ri_(i+1)
% fi_1 == force applied on link {i+1} ???
% Fi_ == force applied on link {i}
% ni_1 ==
% Ni =
% Pci = Point mass of COM
function [fi, ni, torquei]=newtonEulerEQN(joint,R,fi_1, Fi,ni_1,Ni,Pci,P)
    R
    fi=R*fi_1+Fi;

        a=R*ni_1;
        b=cross(Pci,Fi);
        c=cross(P,R*fi_1);
    ni=Ni+a+b+c;
    
    if(joint==0)
        torquei=transpose(ni)*[0;0;1];
    else
        torquei=transpose(fi)*[0;0;1];
    end

end
