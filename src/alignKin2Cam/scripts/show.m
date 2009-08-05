function show(varargin)
% Plot original (uncalibrated) points, axes (if 1st parameter==true)
% and spheres (if 2nd parameter==true) attached to them.
% If the minimization result is given as 3rd parameter, the calibrated
% points according to the found aligning matrices are plotted as well.
%
% Author: Ugo Pattacini

axesGiven=nargin>0;
spheresGiven=nargin>1;
calOn=nargin>2;

Hl=evalin('base','Hl');
Hr=evalin('base','Hr');
Hle=evalin('base','Hle');
Hre=evalin('base','Hre');

len=length(Hl);

% construct aligning matrices
if calOn
    p=varargin{3};
    
    Hxl=axis2dcm(p(1:3));
    Hxl(1:3,4)=p(4:6);
    Hxr=axis2dcm(p(7:9));
    Hxr(1:3,4)=p(10:12);
end

figure; ha=gca;
grid on, axis equal, hold on;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
 
xl1=cell(len,1);
xr1=cell(len,1);
xl2=cell(len,1);
xr2=cell(len,1);

for i=1:len
    % plot original points
    xl1{i}=Hl{i}*Hle{i};
    xr1{i}=Hr{i}*Hre{i};
    hp(1)=plot3(ha,xl1{i}(1,4),xl1{i}(2,4),xl1{i}(3,4),'mo');    
    hp(2)=plot3(ha,xr1{i}(1,4),xr1{i}(2,4),xr1{i}(3,4),'co');
 
    % replot points including the aligning matrices
    % they should appear much more overlapped
    if calOn
        xl2{i}=Hl{i}*Hxl*Hle{i};
        xr2{i}=Hr{i}*Hxr*Hre{i};
        hp(3)=plot3(ha,xl2{i}(1,4),xl2{i}(2,4),xl2{i}(3,4),'ro');
        hp(4)=plot3(ha,xr2{i}(1,4),xr2{i}(2,4),xr2{i}(3,4),'bo');
    end
end

% find a suitable axes length to be plotted
plot3(ha,0,0,0);    % dummy plot for increasing limits before computation
lx=xlim(ha);
ly=ylim(ha);
lz=zlim(ha);
A=0.08*min([abs(lx(2)-lx(1)) abs(ly(2)-ly(1)) abs(lz(2)-lz(1))]);

% plot root reference axes
quiver3(ha,0,0,0,A,0,0,'r','LineWidth',2);
quiver3(ha,0,0,0,0,A,0,'g','LineWidth',2);
quiver3(ha,0,0,0,0,0,A,'b','LineWidth',2);

% plot left eye reference axes
Tl=Hl{1};
quiver3(ha,Tl(1,4),Tl(2,4),Tl(3,4),A*Tl(1,1),A*Tl(2,1),A*Tl(3,1),...
           'r','LineWidth',2);
quiver3(ha,Tl(1,4),Tl(2,4),Tl(3,4),A*Tl(1,2),A*Tl(2,2),A*Tl(3,2),...
           'g','LineWidth',2);
quiver3(ha,Tl(1,4),Tl(2,4),Tl(3,4),A*Tl(1,3),A*Tl(2,3),A*Tl(3,3),...
           'b','LineWidth',2);

% plot right eye reference axes
Tr=Hr{1};
quiver3(ha,Tr(1,4),Tr(2,4),Tr(3,4),A*Tr(1,1),A*Tr(2,1),A*Tr(3,1),...
           'r','LineWidth',2);
quiver3(ha,Tr(1,4),Tr(2,4),Tr(3,4),A*Tr(1,2),A*Tr(2,2),A*Tr(3,2),...
           'g','LineWidth',2);
quiver3(ha,Tr(1,4),Tr(2,4),Tr(3,4),A*Tr(1,3),A*Tr(2,3),A*Tr(3,3),...
           'b','LineWidth',2);

% plot computed vectors orthogonal to image planes
if calOn
    Hipl=Tl*Hxl;
    Hipr=Tr*Hxr;
    hp(5)=quiver3(ha,Hipl(1,4),Hipl(2,4),Hipl(3,4),...
                     2*A*Hipl(1,3),2*A*Hipl(2,3),2*A*Hipl(3,3),'k');
    hp(5)=quiver3(ha,Hipr(1,4),Hipr(2,4),Hipr(3,4),...
                     2*A*Hipr(1,3),2*A*Hipr(2,3),2*A*Hipr(3,3),'k');
end

% Plot axes on the points
if axesGiven
    if varargin{1}
        B=0.25*A;
        for i=1:len
            plotAxes(ha,xl1,xr1,i,B);
            
            if calOn
                plotAxes(ha,xl2,xr2,i,B);
            end
        end

    end
end

% plot the spheres centered in the centers of mass
% and with a radius equal to the mean distance
if spheresGiven
    if varargin{2}
        plotSpheres(ha,xl1,xr1,get(hp(1),'Color'),get(hp(2),'Color'));
        if calOn
            plotSpheres(ha,xl2,xr2,get(hp(3),'Color'),get(hp(4),'Color'));
        end
    end
end

view(3);

Leg={'uncalib. left', 'uncalib. right',...
     'calib. left',   'calib. right',...
     'new Z axis'};
legend(hp,Leg(1:length(hp)),'Location','NorthWest');


%--------------------------------------------------------------------------
function plotSpheres(ha,xl,xr,c1,c2)

len=length(xl);

% find the centers of mass
xlm=zeros(3,len);
xrm=zeros(3,len);
for i=1:len
    xlm(:,i)=xl{i}(1:3,4);
    xrm(:,i)=xr{i}(1:3,4);
end
xlm=mean(xlm,2);
xrm=mean(xrm,2);

% compute the mean distance
d=zeros(len,2);
for i=1:len
    d(i,1)=norm(xl{i}(1:3,4)-xlm);
    d(i,2)=norm(xr{i}(1:3,4)-xrm);
end
dl=mean(d(:,1));
dr=mean(d(:,2));

% plot the spheres
[X,Y,Z]=sphere(12);
Xl=X*dl+xlm(1); Yl=Y*dl+xlm(2); Zl=Z*dl+xlm(3);
Xr=X*dr+xrm(1); Yr=Y*dr+xrm(2); Zr=Z*dr+xrm(3);

hsl=surf(ha,Xl,Yl,Zl);
hsr=surf(ha,Xr,Yr,Zr);
set(hsl,'facecolor',c1);
set(hsr,'facecolor',c2);

% change color transparency
alpha(hsl,0.15);
alpha(hsr,0.15);


%--------------------------------------------------------------------------
function plotAxes(ha,xl,xr,i,A)

% plot x axes
quiver3(ha,xl{i}(1,4),xl{i}(2,4),xl{i}(3,4),...
        A*xl{i}(1,1),A*xl{i}(2,1),A*xl{i}(3,1),'r');
quiver3(ha,xr{i}(1,4),xr{i}(2,4),xr{i}(3,4),...
        A*xr{i}(1,1),A*xr{i}(2,1),A*xr{i}(3,1),'r');

% plot y axes
quiver3(ha,xl{i}(1,4),xl{i}(2,4),xl{i}(3,4),...
        A*xl{i}(1,2),A*xl{i}(2,2),A*xl{i}(3,2),'g');
quiver3(ha,xr{i}(1,4),xr{i}(2,4),xr{i}(3,4),...
        A*xr{i}(1,2),A*xr{i}(2,2),A*xr{i}(3,2),'g');

% plot z axes
quiver3(ha,xl{i}(1,4),xl{i}(2,4),xl{i}(3,4),...
        A*xl{i}(1,3),A*xl{i}(2,3),A*xl{i}(3,3),'b');
quiver3(ha,xr{i}(1,4),xr{i}(2,4),xr{i}(3,4),...
        A*xr{i}(1,3),A*xr{i}(2,3),A*xr{i}(3,3),'b');

    