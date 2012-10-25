function [xhat,yhat,F,H]=compute_ekf_fast(xhat_1,u_1)
%#ok<*NASGU>
%
% Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
% Author: Ugo Pattacini
% CopyPolicy: Released under the terms of the GNU GPL v2.0.

x1=xhat_1(1);
x2=xhat_1(2);
x3=xhat_1(3);
x4=xhat_1(4);

e=exp(-x3/100);

xhat=[x1-(x2*(e-1))/x3+((x4*(x3/100+e-1))/x3^2)*u_1;...
                          x2*e+(-(x4*(e-1))/x3)*u_1;...
                                                 x3;...
                                                 x4];

yhat=x1;

F=[1, -(e-1)/x3, (x2*(e-1))/x3^2+(x2*e)/(100*x3)-(u_1*x4*(e-1)/100)/x3^2-(2*u_1*x4*(x3/100+e-1))/x3^3, (u_1*(x3/100+e-1))/x3^2;...
   0,         e,                                   (u_1*x4*e)/(100*x3)-(x2*e)/100+(u_1*x4*(e-1))/x3^2,         -(u_1*(e-1))/x3;...
   0,         0,                                                                                    1,                       0;...
   0,         0,                                                                                    0,                       1];

H=[1 0 0 0];


