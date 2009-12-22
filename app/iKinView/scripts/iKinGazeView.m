function iKinGazeView
% A viewer for the iCub gaze controlled by a yarp module
% Author: Ugo Pattacini

global R;
global L;
global C;

R{1}.A=0.032;   R{1}.D=0;       R{1}.alpha=pi/2;  R{1}.offset=0;
R{2}.A=0;       R{2}.D=0;       R{2}.alpha=pi/2;  R{2}.offset=-pi/2;
R{3}.A=0.00231; R{3}.D=-0.1933; R{3}.alpha=-pi/2; R{3}.offset=-pi/2;
R{4}.A=0.033;   R{4}.D=0;       R{4}.alpha=pi/2;  R{4}.offset=pi/2;
R{5}.A=0;       R{5}.D=0;       R{5}.alpha=pi/2;  R{5}.offset=pi/2;
R{6}.A=-0.054;  R{6}.D=0.0825;  R{6}.alpha=-pi/2; R{6}.offset=-pi/2;
R{7}.A=0;       R{7}.D=0.034;   R{7}.alpha=-pi/2; R{7}.offset=0;
R{8}.A=0;       R{8}.D=0;       R{8}.alpha=pi/2;  R{8}.offset=-pi/2;

L=R; L{7}.D=-0.034;
C=R; C{7}.D=0;

hfig=figure('Name','iCub Gaze');
title('Select a target point to gaze at');
set(hfig,'Toolbar','figure');
hold on; view(3); grid;
xlim([-0.6 0.2]); xlabel('x [m]');
ylim([-0.6 0.6]); ylabel('y [m]');
zlim([-0.6 0.6]); zlabel('z [m]');

hax=get(hfig,'CurrentAxes');
set(hax,'DataAspectRatio',[1 1 1]);

lim=axis;
A=max(abs(lim))*0.1;
    
quiver3(0,0,0,A,0,0,'Color','r','Linewidth',2);
quiver3(0,0,0,0,A,0,'Color','g','Linewidth',2);
quiver3(0,0,0,0,0,A,'Color','b','Linewidth',2);

q=zeros(9,1);
[xR,axR]=fkin('r',q);
[xL,axL]=fkin('l',q);
[xC,axC]=fkin('c',q);
hg1=drawGaze(hfig,xR,xL,xC,axR,axL,axC);

assignin('base','xd',[]);   % init with empty value
xd=[0 0 0];
tg=plot3(xd(1),xd(2),xd(3),'ro','MarkerFaceColor','r');

hg2=hgtransform('Parent',hax);
set(tg,'Parent',hg2);

t=timer('Period',0.05,'ExecutionMode','fixedRate',...
        'TimerFcn',@PlotGaze,'StopFcn',@StopFcn,...
        'UserData',{hfig, hg1 q});

modeOnce=true;
xd_ml=xd;
set(hfig,'UserData',{t modeOnce xd xd_ml hg2});

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
      
uicontrol(hfig,'Style','Text','Position',[10 300-30*3 60 15],...
          'String','At once');      
      
uicontrol(hfig,'Style','Checkbox','Position',[70 300-30*3 16 16],...
          'Value',true,'Callback',@SwitchMode);
      
uicontrol(hfig,'Style','Pushbutton','Position',[10 300-30*4 80 20],...
          'String','Apply Target','Callback',@ApplyTarget);

uicontrol(hfig,'Style','Pushbutton','Position',[10 300-30*5 80 20],...
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

sel=2:10;

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
for i=1:length(sel)
    lgnd{i}=sprintf('q_%d',i);
end
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
H=makehgtform('translate',xd_ml);
xd=xd_ml(1:3);
          
if ~modeOnce
    assignin('base','xd',xd');
end

set(hg,'Matrix',H);
drawnow;

UserData{3}=xd;
UserData{4}=xd_ml;
set(hfig,'UserData',UserData);


%--------------------------------------------------------------------------
function T=DH(P,n,theta)

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
function [x,ax]=fkin(type,q)

global R;
global L;
global C;

q=q*pi/180;

if strcmpi(type,'r')
    P=R;
    q(8)=q(8)-q(9)/2;
    alignH=evalin('base','alignHR');
elseif strcmpi(type,'l')
    P=L;
    q(8)=q(8)+q(9)/2;
    alignH=evalin('base','alignHL');
else
    P=C;
    q(7)=0;
    q(8)=0;
    alignH=eye(4,4);
end        

T=cell(8+1,1);
x=cell(8+1,1);
T{1}=[[0 -1 0 0]; [0 0 -1 0]; [1 0 0 0]; [0 0 0 1]];
x{1}=T{1}(1:3,4);

for i=1:8
    T{i+1}=T{i}*DH(P,i,q(i));
    x{i+1}=T{i+1}(1:3,4);
end

% add aligning matrix
T{end}=T{end}*alignH;
x{end}=T{end}(1:3,4);

ax{1}=T{end}(1:3,1);
ax{2}=T{end}(1:3,2);
ax{3}=T{end}(1:3,3);


%--------------------------------------------------------------------------
function hg=drawGaze(hfig,xR,xL,xC,axR,axL,axC)

set(0,'CurrentFigure',hfig);
hax=get(hfig,'CurrentAxes');
lim=axis(hax);
M=max(abs(lim));
A=M*0.025;

eyeR=plot3(hax,[xR{1}(1) xR{2}(1) xR{3}(1) xR{4}(1) xR{5}(1) xR{6}(1) xR{7}(1) xR{8}(1) xR{9}(1)],...
               [xR{1}(2) xR{2}(2) xR{3}(2) xR{4}(2) xR{5}(2) xR{6}(2) xR{7}(2) xR{8}(2) xR{9}(2)],...
               [xR{1}(3) xR{2}(3) xR{3}(3) xR{4}(3) xR{5}(3) xR{6}(3) xR{7}(3) xR{8}(3) xR{9}(3)],...
               'Color','k','LineWidth',3);
     
eyeL=plot3(hax,[xL{1}(1) xL{2}(1) xL{3}(1) xL{4}(1) xL{5}(1) xL{6}(1) xL{7}(1) xL{8}(1) xL{9}(1)],...
               [xL{1}(2) xL{2}(2) xL{3}(2) xL{4}(2) xL{5}(2) xL{6}(2) xL{7}(2) xL{8}(2) xL{9}(2)],...
               [xL{1}(3) xL{2}(3) xL{3}(3) xL{4}(3) xL{5}(3) xL{6}(3) xL{7}(3) xL{8}(3) xL{9}(3)],...
               'Color','k','LineWidth',3);

axR_(1)=quiver3(hax,xR{end}(1),xR{end}(2),xR{end}(3),...
              A*axR{1}(1),A*axR{1}(2),A*axR{1}(3),...
              'Color','r','Linewidth',2);
axR_(2)=quiver3(hax,xR{end}(1),xR{end}(2),xR{end}(3),...
              A*axR{2}(1),A*axR{2}(2),A*axR{2}(3),...
              'Color','g','Linewidth',2);
axR_(3)=quiver3(hax,xR{end}(1),xR{end}(2),xR{end}(3),...
              A*axR{3}(1),A*axR{3}(2),A*axR{3}(3),...
              'Color','b','Linewidth',2);
          
axR_(4)=quiver3(hax,xR{end}(1),xR{end}(2),xR{end}(3),...
                M*axR{3}(1),M*axR{3}(2),M*axR{3}(3),...
               'Color','m','Linewidth',1);

axL_(1)=quiver3(hax,xL{end}(1),xL{end}(2),xL{end}(3),...
              A*axL{1}(1),A*axL{1}(2),A*axL{1}(3),...
              'Color','r','Linewidth',2);
axL_(2)=quiver3(hax,xL{end}(1),xL{end}(2),xL{end}(3),...
              A*axL{2}(1),A*axL{2}(2),A*axL{2}(3),...
              'Color','g','Linewidth',2);
axL_(3)=quiver3(hax,xL{end}(1),xL{end}(2),xL{end}(3),...
              A*axL{3}(1),A*axL{3}(2),A*axL{3}(3),...
              'Color','b','Linewidth',2);
          
axL_(4)=quiver3(hax,xL{end}(1),xL{end}(2),xL{end}(3),...
                M*axL{3}(1),M*axL{3}(2),M*axL{3}(3),...
               'Color','m','Linewidth',1);

axC_(1)=quiver3(hax,xC{end}(1),xC{end}(2),xC{end}(3),...
              A*axC{1}(1),A*axC{1}(2),A*axC{1}(3),...
              'Color','r','Linewidth',2);
axC_(2)=quiver3(hax,xC{end}(1),xC{end}(2),xC{end}(3),...
              A*axC{2}(1),A*axC{2}(2),A*axC{2}(3),...
              'Color','g','Linewidth',2);
axC_(3)=quiver3(hax,xC{end}(1),xC{end}(2),xC{end}(3),...
              A*axC{3}(1),A*axC{3}(2),A*axC{3}(3),...
              'Color','b','Linewidth',2);
          
axC_(4)=quiver3(hax,xC{end}(1),xC{end}(2),xC{end}(3),...
                M*axC{3}(1),M*axC{3}(2),M*axC{3}(3),...
               'Color','b','Linewidth',1);

P0a=xR{end};
P1a=axR{3};
P0b=xL{end};
P1b=axL{3};

tmp1=dot(P1a,P1b);
tmp2=P0b-P0a;
tmp3=tmp1*tmp1-1;

ta=dot(tmp1*P1b-P1a,tmp2) / tmp3;
tb=dot(P1b-tmp1*P1a,tmp2) / tmp3;

Pa=P0a+ta*P1a;
Pb=P0b+tb*P1b;
P=(Pa+Pb)/2;

if max(abs(P))<inf
    fixationPoint=plot3(hax,P(1),P(2),P(3),'m*');
else
    fixationPoint=[];
end
           
hg=hggroup;
set(eyeR,'Parent',hg);
set(eyeL,'Parent',hg);
set(axR_,'Parent',hg);
set(axL_,'Parent',hg);
set(axC_,'Parent',hg);
set(fixationPoint,'Parent',hg);


%--------------------------------------------------------------------------
function PlotGaze(obj,event,string_arg) %#ok<INUSD>

UserData=get(obj,'UserData');

hfig =UserData{1};
hg   =UserData{2};
q_old=UserData{3};

try
    q=evalin('base','Buffer(idx,2:10)')';
catch %#ok<CTCH>
    q=q_old;
end
    
if any(q~=q_old)
    [xR,axR]=fkin('r',q);
    [xL,axL]=fkin('l',q);
    [xC,axC]=fkin('c',q);

    if ~isempty(hg)
        delete(hg)
    end

    hg=drawGaze(hfig,xR,xL,xC,axR,axL,axC);
    drawnow;
end

UserData{2}=hg;
UserData{3}=q;
set(obj,'UserData',UserData);


%--------------------------------------------------------------------------
function StopFcn(obj,event,string_arg) %#ok<INUSD>

delete(obj);

