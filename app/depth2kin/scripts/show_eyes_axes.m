function show_eyes_axes(Tl,Tr)

figure('color','white');
view(3);
grid on;
hold on;

Tl0=eye(4,4); Tl0(1,4)=-0.034;
Tr0=eye(4,4); Tr0(1,4)=+0.034;

show_eye(Tl0*eye(4,4),Tl0*Tl);
show_eye(Tr0*eye(4,4),Tr0*Tr);


function show_eye(To,Tn)

scale=0.005;

% old axes
x=To(1:3,1);
y=To(1:3,2);
z=To(1:3,3);
p=To(1:3,4);

quiver3(p(1),p(2),p(3),p(1)+x(1),p(2)+x(2),p(3)+x(3),scale,'r--','linewidth',2.0);
quiver3(p(1),p(2),p(3),p(1)+y(1),p(2)+y(2),p(3)+y(3),scale,'g--','linewidth',2.0);
quiver3(p(1),p(2),p(3),p(1)+z(1),p(2)+z(2),p(3)+z(3),scale,'b--','linewidth',2.0);

% new axes
x=Tn(1:3,1);
y=Tn(1:3,2);
z=Tn(1:3,3);
p=Tn(1:3,4);

quiver3(p(1),p(2),p(3),p(1)+x(1),p(2)+x(2),p(3)+x(3),scale,'r','linewidth',2.0);
quiver3(p(1),p(2),p(3),p(1)+y(1),p(2)+y(2),p(3)+y(3),scale,'g','linewidth',2.0);
quiver3(p(1),p(2),p(3),p(1)+z(1),p(2)+z(2),p(3)+z(3),scale,'b','linewidth',2.0);

axis equal;
