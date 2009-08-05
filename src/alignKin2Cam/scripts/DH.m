function T=DH(P,n,theta)
% Returns the Denavit-Hartenberg rototranslation matrix 
%
% Used by the minimization process
%
% Author: Ugo Pattacini

theta=theta+P{n}.offset;
c_theta=cos(theta);
s_theta=sin(theta);
c_alpha=cos(P{n}.alpha);
s_alpha=sin(P{n}.alpha);

T=[[c_theta -s_theta*c_alpha  s_theta*s_alpha P{n}.A*c_theta];...
   [s_theta  c_theta*c_alpha -c_theta*s_alpha P{n}.A*s_theta];...
   [      0          s_alpha          c_alpha         P{n}.D];...
   [      0                0                0              1]];

