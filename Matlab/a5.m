syms h_0 h_1 
syms v_1 
syms delta0 delta1

m1 = [2/h_0 4/h_0+4/h_1 2/h_1];
m1v = [0; v_1; 0];
m1a = [6*delta0/h_0^2+6*delta1/h_1^2];

m1Sol = solve(m1*m1v == m1a,v_1);


syms a0 b0 c0 d0
syms y_0 y_1
v0 =0;
a0 = y_0;
b0 = v_0*h_0;
c0 = 3*(y_1 - y_0) - 2*(v_0*h_0) - v_1*h_0;
d0 = -2*(y_1-y_0) + v_0*h_0 + v_1*h_0;

y0 = 