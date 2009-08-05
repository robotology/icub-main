function T=fkin(type,q)
% Returns the kinematic rototranslation matrices 
% for iCub left and right eye
%
% Used by the minimization process
%
% Author: Ugo Pattacini

P{1}.A=0.032;   P{1}.D=0;       P{1}.alpha=pi/2;  P{1}.offset=0;
P{2}.A=0;       P{2}.D=0;       P{2}.alpha=pi/2;  P{2}.offset=-pi/2;
P{3}.A=0.00231; P{3}.D=-0.1933; P{3}.alpha=-pi/2; P{3}.offset=-pi/2;
P{4}.A=0.033;   P{4}.D=0;       P{4}.alpha=pi/2;  P{4}.offset=pi/2;
P{5}.A=0;       P{5}.D=0;       P{5}.alpha=pi/2;  P{5}.offset=pi/2;
P{6}.A=-0.054;  P{6}.D=0.0825;  P{6}.alpha=-pi/2; P{6}.offset=-pi/2;
P{7}.A=0;       P{7}.D=0.034;   P{7}.alpha=-pi/2; P{7}.offset=0;
P{8}.A=0;       P{8}.D=0;       P{8}.alpha=pi/2;  P{8}.offset=-pi/2;

if strcmpi(type,'r')
    q(8)=q(8)-q(9)/2;
else
    P{7}.D=-0.034;
    q(8)=q(8)+q(9)/2;
end        

T=[[0 -1 0 0]; [0 0 -1 0]; [1 0 0 0]; [0 0 0 1]];

for i=1:8
    T=T*DH(P,i,q(i));
end
