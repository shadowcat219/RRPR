%% Case 1: 3 Points: Start -> Via1 -> Goal (i.e. m = 2, thus 1 eqt)
syms h_0 h_1 
syms v_1 
syms delta0 delta1

m1 = [2/h_0 4/h_0+4/h_1 2/h_1]
m1v = [0; v_1; 0]
m1a = [6*delta0/h_0^2+6*delta1/h_1^2]

m1Sol = solve(m1*m1v == m1a,v_1); %Solves for v_1

%% Case 2: 4 Points: Start -> Via1 -> Via2 -> Goal (m=3, 2 eqts)

syms h_2
syms v_2 
syms delta2

m2 = [2/h_0 4/h_0+4/h_1 2/h_1 0; 0 2/h_1 4/h_1+4/h_2 2/h_2];
m2v = [0; v_1; v_2; 0];
m2a = [6*delta0/h_0^2+6*delta1/h_1^2; 6*delta1/h_1^2+6*delta2/h_2^2];

m2v_1 = solve(m2(1,:)*m2v==m2a(1,:), v_1); %v_1 = f(v_2)
m2v_2 = solve(m2(2,:)*m2v==m2a(2,:), v_2); %v_2 = f(v_1)

m2Sol_temp1 = subs(m2v_2, v_1, m2v_1);
m2Sol_v2 = simplifyFraction(solve(m2Sol_temp1==v_2, v_2)); %solves for v2
m2Sol_v1 = subs(m2v_1, v_2, m2Sol_v1); %solves for v1

%% Case 3: 5 Points: Start -> Via1 -> Via2 -> Via3 -> Goal (m=4, 3 eqts)

syms h_3
syms v_3
syms delta3

m3 = [2/h_0 4/h_0+4/h_1 2/h_1 0 0; 0 2/h_1 4/h_1+4/h_2 2/h_2 0; 0 0 2/h_2 4/h_2+4/h_3 2/h_3];
m3v = [0; v_1; v_2; v_3; 0];
m3a = [6*delta0/h_0^2+6*delta1/h_1^2; 6*delta1/h_1^2+6*delta2/h_2^2; 6*delta3/h_2^2+6*delta3/h_3^2];

m3v_a = solve(m3(1,:)*m3v==m3a(1,:), v_1); % v_1 = f(v_2)
m3v_b = solve(m3(3,:)*m3v==m3a(3,:), v_3); % v_3 = f(v_2)
m3v_c = solve(m3(2,:)*m3v==m3a(2,:), v_2); % v_2 = f(v_1, v_3)

m3v_temp1 = subs(m3v_c, v_1, m3v_a); %v_2 = f(v_3)
m3v_temp2 = subs(m3v_temp1, v_3, m3v_b); 
m3Sol_v2 = simplifyFraction(solve(m3v_temp2==v_2, v_2)); %solves for v2
m3sol_temp3 = subs(m3v_b, v_2, m3Sol_v2);
m3Sol_v3 = simplifyFraction(solve(m3sol_temp3==v_3, v_3)); %solves for v3
m3sol_temp4 = subs(m3v_a, v_2, m3Sol_v2);
m3Sol_v1 = simplifyFraction(solve(m3sol_temp4==v_1, v_1)); %solves for v1