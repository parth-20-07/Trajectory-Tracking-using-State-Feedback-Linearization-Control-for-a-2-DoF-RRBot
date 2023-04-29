%% Two-Link Planar Robot Manipulator with Distributed Mass
%% Symbolic Definition

syms g m1 m2 h1 h2 v1 v2 I1 I2 w1 w2 l1 l2 r1 r2 x1(t) x2(t) y1(t) y2(t) theta1(t) theta2(t) T1 T2 'real' % Creating Symbolic Variables
sympref('AbbreviateOutput', false);
sympref('MatrixWithSquareBrackets', true);
sympref('PolynomialDisplayStyle', 'ascend');
%% Define Generalised Coordinates

q = [theta1; theta2];
dq = formula(diff(q, t));
%% Defining Generalised Inputs

u = [T1; T2];
%% Derive Lagrangian Function

PE = m1 * g * h1 + m2 * g * h2;
PE = subs(PE, [h1, h2], [r1 * cos(theta1), l1 * cos(theta1) + r2 * cos(theta1 + theta2)]);
KE = (m1 * (v1^2) / 2) + (I1 * (w1^2) / 2) + (m2 * (v2^2) / 2) + (I2 * (w2^2) / 2);
KE = subs(KE, [v1, v2], [sqrt(diff(x1, t)^2 + diff(y1, t)^2), sqrt(diff(x2, t)^2 + diff(y2, t)^2)]);
KE = subs(KE, [x1, y1, w1, x2, y2, w2], [r1 * sin(theta1), r1 * cos(theta1), diff(theta1, t), l1 * sin(theta1) + r2 * sin(theta1 + theta2), l1 * cos(theta1) + r2 * cos(theta1 + theta2), diff(theta1 + theta2, t)]);
LE = KE - PE;
LE = simplify(LE);
%% Derive Euler Lagrangian Equations

dLE_dq = formula(jacobian(LE, q));
dLE_ddq = formula(jacobian(LE, dq));
d_dLE_dtDdq = formula(diff(dLE_ddq, t));
EU = formula(d_dLE_dtDdq - dLE_dq - u');
EU = simplify(EU);
syms thethaddot1 thethaddot2 'real'
EU = subs(EU, [diff(theta1, t, 2), diff(theta2, t, 2)], [thethaddot1, thethaddot2]);
%% Symbolic Solution to EL Equation

[thethaddot1, thethaddot2] = solve(EU, [thethaddot1, thethaddot2]);
ddq = formula([diff(theta1, t, 2) - thethaddot1; diff(theta2, t, 2) - thethaddot2]);
