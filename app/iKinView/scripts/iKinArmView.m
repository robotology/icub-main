function iKinArmView(varargin)
% A viewer for the iCub arm controlled by a yarp module
% Author: Ugo Pattacini

if nargin
    armType=varargin{1};
    if (~strcmpi(armType,'right') && ~strcmpi(armType,'left'))
        armType='right';
    end
else
    armType='right';
end

global P;

if strcmpi(armType,'right')
    P{1}.A =0.032;      P{1}.D =0;        P{1}.alpha =pi/2;  P{1}.offset =0;
    P{2}.A =0;          P{2}.D =0;        P{2}.alpha =pi/2;  P{2}.offset =-pi/2;
    P{3}.A =-0.0233647; P{3}.D =-0.1433;  P{3}.alpha =pi/2;  P{3}.offset =-pi/2-15*pi/180;
    P{4}.A =0;          P{4}.D =-0.10774; P{4}.alpha =pi/2;  P{4}.offset =-pi/2;
    P{5}.A =0;          P{5}.D =0;        P{5}.alpha =-pi/2; P{5}.offset =-pi/2;
    P{6}.A =0;          P{6}.D =-0.15228; P{6}.alpha =-pi/2; P{6}.offset =-pi/2-15*pi/180;
    P{7}.A =0.015;      P{7}.D =0;        P{7}.alpha =pi/2;  P{7}.offset =0;
    P{8}.A =0;          P{8}.D =-0.1373;  P{8}.alpha =pi/2;  P{8}.offset =-pi/2;
    P{9}.A =0;          P{9}.D =0;        P{9}.alpha =pi/2;  P{9}.offset =pi/2;
    P{10}.A=0.0625;     P{10}.D=0.016;    P{10}.alpha=0;     P{10}.offset=pi;
else
    P{1}.A =0.032;      P{1}.D =0;        P{1}.alpha =pi/2;  P{1}.offset =0;
    P{2}.A =0;          P{2}.D =0;        P{2}.alpha =pi/2;  P{2}.offset =-pi/2;
    P{3}.A =0.0233647;  P{3}.D =-0.1433;  P{3}.alpha =-pi/2; P{3}.offset =pi/2+15*pi/180;
    P{4}.A =0;          P{4}.D =0.10774;  P{4}.alpha =-pi/2; P{4}.offset =pi/2;
    P{5}.A =0;          P{5}.D =0;        P{5}.alpha =pi/2;  P{5}.offset =-pi/2;
    P{6}.A =0;          P{6}.D =0.15228;  P{6}.alpha =-pi/2; P{6}.offset =pi/2-15*pi/180;
    P{7}.A =-0.015;     P{7}.D =0;        P{7}.alpha =pi/2;  P{7}.offset =0;
    P{8}.A =0;          P{8}.D =0.1373;   P{8}.alpha =pi/2;  P{8}.offset =-pi/2;
    P{9}.A =0;          P{9}.D =0;        P{9}.alpha =pi/2;  P{9}.offset =pi/2;
    P{10}.A=0.0625;     P{10}.D=-0.016;   P{10}.alpha=0;     P{10}.offset=0;
end

hfig=figure('Name','iCub Arm');
title('Select a target point to move to');
set(hfig,'Toolbar','figure');
hold on; view(3); grid;
xlim([-0.6 0.2]); xlabel('x [m]');
ylim([-0.6 0.6]); ylabel('y [m]');
zlim([-0.6 0.6]); zlabel('z [m]');

hax=get(hfig,'CurrentAxes');
set(hax,'DataAspectRatio',[1 1 1]);

lim=axis;
A=max(abs(lim))*0.2;
    
quiver3(0,0,0,A/2,0,0,'Color','r','Linewidth',2);
quiver3(0,0,0,0,A/2,0,'Color','g','Linewidth',2);
quiver3(0,0,0,0,0,A/2,'Color','b','Linewidth',2);

q=zeros(10,1);
[x,axpoint]=fkin(q);
hg1=drawArm(hfig,x,axpoint);

assignin('base','xd',[]);   % init with empty value
xd   =[0 0 0 0 0 0 0];
xd_ml=[0 0 0 0 0 0];
ax(1)=quiver3(0,0,0,A,0,0,'Color','r','Linewidth',2);
ax(2)=quiver3(0,0,0,0,A,0,'Color','g','Linewidth',2);
ax(3)=quiver3(0,0,0,0,0,A,'Color','b','Linewidth',2);

hg2=hgtransform('Parent',hax);
set(ax,'Parent',hg2);

t=timer('Period',0.05,'ExecutionMode','fixedRate',...
        'TimerFcn',@PlotArm,'StopFcn',@StopFcn,...
        'UserData',{hfig, hg1 q});

modeOnce=true;
set(hfig,'UserData',{t modeOnce xd xd_ml hg2});

uicontrol(hfig,'Style','Pushbutton','Position',[10 300+60 80 20],...
          'String','Align Target','Callback',@AlignTarget);

uicontrol(hfig,'Style','Text','Position',[10 300+25 80 15],...
          'String','Target sliders');

uicontrol(hfig,'Style','Slider','Position',[10 300 80 20],...
          'Min',lim(1),'Max',lim(2),'Value',0,...
          'Callback',@SelectTarget,'UserData',{1});
      
uicontrol(hfig,'Style','Slider','Position',[10 300-30 80 20],...
          'Min',lim(3),'Max',lim(4),'Value',0,...
          'Callback',@SelectTarget,'UserData',{2});

uicontrol(hfig,'Style','Slider','Position',[10 300-30*2 80 20],...
          'Min',lim(5),'Max',lim(6),'Value',0,...
          'Callback',@SelectTarget,'UserData',{3});

uicontrol(hfig,'Style','Slider','Position',[10 300-30*3 80 20],...
          'Min',-180,'Max',180,'Value',0,...
          'Callback',@SelectTarget,'UserData',{4});

uicontrol(hfig,'Style','Slider','Position',[10 300-30*4 80 20],...
          'Min',-180,'Max',180,'Value',0,...
          'Callback',@SelectTarget,'UserData',{5});

uicontrol(hfig,'Style','Slider','Position',[10 300-30*5 80 20],...
          'Min',-180,'Max',180,'Value',0,...
          'Callback',@SelectTarget,'UserData',{6});
      
uicontrol(hfig,'Style','Text','Position',[10 300-30*6 60 15],...
          'String','At once');      
      
uicontrol(hfig,'Style','Checkbox','Position',[70 300-30*6 16 16],...
          'Value',true,'Callback',@SwitchMode);
      
uicontrol(hfig,'Style','Pushbutton','Position',[10 300-30*7 80 20],...
          'String','Apply Target','Callback',@ApplyTarget);

uicontrol(hfig,'Style','Pushbutton','Position',[10 300-30*8 80 20],...
          'String','Plot Traj.','Callback',@PlotData);    
          
set(hfig,'CloseRequestFcn',@Quit);

start(t);
      

%--------------------------------------------------------------------------
function Quit(src,eventdata) %#ok<INUSD>

UserData=get(src,'UserData');
t=UserData{1};

stop(t);
delete(src);


%--------------------------------------------------------------------------
function SwitchMode(src,eventdata) %#ok<INUSD>

hfig=get(src,'Parent');
hdl=findobj(hfig,'Style','Pushbutton','String','Apply Target');
UserData=get(hfig,'UserData');

state=get(src,'Value');

if state
    set(hdl,'Enable','on');
else
    set(hdl,'Enable','off');
end

UserData{2}=state;
set(hfig,'UserData',UserData);


%--------------------------------------------------------------------------
function ApplyTarget(src,evendata) %#ok<INUSD>

hfig=get(src,'Parent');
UserData=get(hfig,'UserData');
xd=UserData{3};

assignin('base','xd',xd');


%--------------------------------------------------------------------------
function PlotData(src,evendata) %#ok<INUSD>

try
    Buffer=evalin('base','Buffer');
    idx=evalin('base','idx');
catch %#ok<CTCH>
    warndlg('Nothing to display');
    return;
end

sel=2:11;

if idx<length(Buffer)
    time=[Buffer(idx+1:end,1); Buffer(1:idx,1)];
    Data=[Buffer(idx+1:end,sel); Buffer(1:idx,sel)];
else
    time=Buffer(:,1);
    Data=Buffer(:,sel);
end

figure('Color','w');
hdl=plot(time,Data,'LineWidth',2);
grid;
lgnd=cell(10,1);
for i=1:9
    lgnd{i}=sprintf('q_%d',i);
end
lgnd{10}='q_1_0';
legend(hdl,lgnd);
xlabel(hdl,sprintf('Time [s]'));


%--------------------------------------------------------------------------
function SelectTarget(src,eventdata) %#ok<INUSD>

UserData=get(src,'UserData');
idx=UserData{1};

hfig=get(src,'Parent');
UserData=get(hfig,'UserData');
modeOnce=UserData{2};
xd_ml   =UserData{4};
hg      =UserData{5};

xd_ml(idx)=get(src,'Value');
r=xd_ml(4:6)*pi/180;
n=angle2dcm(r(1),r(2),r(3),'ZYX');
v=dcm2axis(n);

if ~any(v(1:3))
    v(1:3)=[1 0 0];
end

H=makehgtform('translate',xd_ml(1:3),...
              'axisrotate',v(1:3),v(4));
          
xd(1:3)=xd_ml(1:3);
xd(4:7)=v;
          
if ~modeOnce
    assignin('base','xd',xd');
end

set(hg,'Matrix',H);
drawnow;

UserData{3}=xd;
UserData{4}=xd_ml;
set(hfig,'UserData',UserData);


%--------------------------------------------------------------------------
function AlignTarget(src,evendata) %#ok<INUSD>

hfig=get(src,'Parent');
UserDataFig=get(hfig,'UserData');
t       =UserDataFig{1};
modeOnce=UserDataFig{2};
xd_ml   =UserDataFig{4};
hg      =UserDataFig{5};

UserDataTimer=get(t,'UserData');
q_old=UserDataTimer{3};

try
    q=evalin('base','Buffer(idx,2:11)')';
catch %#ok<CTCH>
    q=q_old;
end

[x,axpoint]=fkin(q);
H=[axpoint{1} axpoint{2} axpoint{3}];
v=dcm2axis(H);
[r1,r2,r3]=dcm2angle(H,'ZYX');

xd_ml(1:3)=x{end}';
xd_ml(4:6)=[r1 r2 r3]*180/pi;
         
if ~any(v(1:3))
    v(1:3)=[1 0 0];
end

H=makehgtform('translate',xd_ml(1:3),...
              'axisrotate',v(1:3),v(4));
   
set(hg,'Matrix',H);
drawnow;

sld=findobj(hfig,'Style','Slider');
for i=1:length(sld)
    idx=get(sld(i),'UserData');
    set(sld(i),'Value',xd_ml(idx{1}));
end

xd(1:3)=xd_ml(1:3)*0.001;
xd(4:7)=v;
          
if ~modeOnce
    assignin('base','xd',xd');
end

UserDataFig{3}=xd;
UserDataFig{4}=xd_ml;
set(hfig,'UserData',UserDataFig);


%--------------------------------------------------------------------------
function v=dcm2axis(R)

v(1)=R(3,2)-R(2,3);
v(2)=R(1,3)-R(3,1);
v(3)=R(2,1)-R(1,2);
r=norm(v);

if ~r
    v=zeros(1,4);
    return;
end

theta=atan2(0.5*r,0.5*(R(1,1)+R(2,2)+R(3,3)-1));

v=v/r;
v(4)=theta;


%--------------------------------------------------------------------------
function T=DH(n,theta)

global P;

theta=theta+P{n}.offset;
c_theta=cos(theta);
s_theta=sin(theta);
c_alpha=cos(P{n}.alpha);
s_alpha=sin(P{n}.alpha);

T=[[c_theta -s_theta*c_alpha  s_theta*s_alpha P{n}.A*c_theta];...
   [s_theta  c_theta*c_alpha -c_theta*s_alpha P{n}.A*s_theta];...
   [      0          s_alpha          c_alpha         P{n}.D];...
   [      0                0                0              1]];


%--------------------------------------------------------------------------
function [x,axpoint]=fkin(q)

q=q*pi/180;

T0 =[[0 -1 0 0]; [0 0 -1 0]; [1 0 0 0]; [0 0 0 1]];
T1 =T0*DH(1,q(1));
T2 =T1*DH(2,q(2));
T3 =T2*DH(3,q(3));
T4 =T3*DH(4,q(4));
T5 =T4*DH(5,q(5));
T6 =T5*DH(6,q(6));
T7 =T6*DH(7,q(7));
T8 =T7*DH(8,q(8));
T9 =T8*DH(9,q(9));
T10=T9*DH(10,q(10));

x{1} =T0(1:3,4);
x{2} =T1(1:3,4);
x{3} =T2(1:3,4);
x{4} =T3(1:3,4);
x{5} =T4(1:3,4);
x{6} =T5(1:3,4);
x{7} =T6(1:3,4);
x{8} =T7(1:3,4);
x{9} =T8(1:3,4);
x{10}=T9(1:3,4);
x{11}=T10(1:3,4);

axpoint{1}=T10(1:3,1);
axpoint{2}=T10(1:3,2);
axpoint{3}=T10(1:3,3);


%--------------------------------------------------------------------------
function hg=drawArm(hfig,x,axpoint)

set(0,'CurrentFigure',hfig);
hax=get(hfig,'CurrentAxes');
lim=axis(hax);
A=max(abs(lim))*0.2;

arm=plot3(hax,[x{1}(1) x{2}(1) x{3}(1) x{4}(1) x{5}(1) x{6}(1) x{7}(1) x{8}(1) x{9}(1) x{10}(1) x{11}(1)],...
              [x{1}(2) x{2}(2) x{3}(2) x{4}(2) x{5}(2) x{6}(2) x{7}(2) x{8}(2) x{9}(2) x{10}(2) x{11}(2)],...
              [x{1}(3) x{2}(3) x{3}(3) x{4}(3) x{5}(3) x{6}(3) x{7}(3) x{8}(3) x{9}(3) x{10}(3) x{11}(3)],...
              'Color','k','LineWidth',3);
     
axpoint{1}=axpoint{1}*A;
axpoint{2}=axpoint{2}*A;
axpoint{3}=axpoint{3}*A;

ax(1)=quiver3(hax,x{end}(1),x{end}(2),x{end}(3),...
              axpoint{1}(1),axpoint{1}(2),axpoint{1}(3),...
              'Color','r','Linewidth',2);
ax(2)=quiver3(hax,x{end}(1),x{end}(2),x{end}(3),...
              axpoint{2}(1),axpoint{2}(2),axpoint{2}(3),...
              'Color','g','Linewidth',2);
ax(3)=quiver3(hax,x{end}(1),x{end}(2),x{end}(3),...
              axpoint{3}(1),axpoint{3}(2),axpoint{3}(3),...
              'Color','b','Linewidth',2);

hg=hggroup;
set(arm,'Parent',hg);
set(ax,'Parent',hg);


%--------------------------------------------------------------------------
function PlotArm(obj,event,string_arg) %#ok<INUSD>

UserData=get(obj,'UserData');

hfig =UserData{1};
hg   =UserData{2};
q_old=UserData{3};

try
    q=evalin('base','Buffer(idx,2:11)')';
catch %#ok<CTCH>
    q=q_old;
end
    
if any(q~=q_old)
    [x,axpoint]=fkin(q);

    if ~isempty(hg)
        delete(hg)
    end

    hg=drawArm(hfig,x,axpoint);
    drawnow;
end

UserData{2}=hg;
UserData{3}=q;
set(obj,'UserData',UserData);


%--------------------------------------------------------------------------
function StopFcn(obj,event,string_arg) %#ok<INUSD>

delete(obj);

