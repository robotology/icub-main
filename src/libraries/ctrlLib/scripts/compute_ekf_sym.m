function [xhat,yhat,F,H]=compute_ekf_sym(xhat_1,u_1)
%#ok<*NASGU>
%
% Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
% Author: Ugo Pattacini
% CopyPolicy: Released under the terms of the GNU GPL v2.0.

x1=sym('x1');
x2=sym('x2');
x3=sym('x3');
x4=sym('x4');
T=0.01;

Ad=[1 (1-exp(-x3*T))/x3; 0 exp(-x3*T)];
Bd=x4*[(exp(-x3*T)+x3*T-1)/(x3*x3); (1-exp(-x3*T))/x3];

A_aug=[Ad zeros(2,2); zeros(2,2) eye(2,2)];
B_aug=[Bd;0;0];
C_aug=[1 0 0 0];

x=[x1;x2;x3;x4];
f=A_aug*x+B_aug*u_1;
h=C_aug*x;

F=jacobian(f,x);
H=jacobian(h,x);

x1=subs(x1,xhat_1(1));
x2=subs(x2,xhat_1(2));
x3=subs(x3,xhat_1(3));
x4=subs(x4,xhat_1(4));

xhat=subs(f);
yhat=subs(h);
F=subs(F);
H=subs(H);


