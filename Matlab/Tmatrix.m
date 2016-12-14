function T=Tmatrix(theta, alpha_1, a_1, d)

T=[cos(theta), -sin(theta), 0, a_1;
sin(theta)*cos(alpha_1), cos(theta)*cos(alpha_1), -sin(alpha_1), -sin(alpha_1)*d;
sin(theta)*sin(alpha_1), cos(theta)*sin(alpha_1), cos(alpha_1), cos(alpha_1)*d;
0, 0, 0 1];
end

