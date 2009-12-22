function varargout = stripchart(varargin)
% STRIPCHART Create a stripchart for real-time data display
%
% STRIPCHART makes it fairly easy to include a strip chart in your
% real-time data acquisition and analysis application.  You feed
% stripchart your data, and it updates the display.  It takes
% 2 steps to use STRIPCHART.  First, you initialize the stripchart with basic
% information needed for the FFT (sample rate, axis width, number of traces).
% After that, all you need to do is pass your data to the scope to update.
%
% This documentation starts with the simplest syntax for the two steps,
% then provides a few more advanced options.
%
% STEP 1: Initialize the stripchart
% STRIPCHART(FS,AXESWIDTH) initializes a stripchart in the current axes.
% This stripchart will display incoming data with a sample rate of FS Hz.
% The stripchart will display the most recent AXESWIDTH seconds of data.
%
% STEP 2: Update the scope
% STRIPCHART(S) updates the stripchart in the current axes with the
% data in vector S.  The scope should first be initialized as above with
% sample rate and axes width.  If not, the sample rate will be 1 Hz and the
% axes width will match the duration of S.  
%
% STRIPCHART(FS,AXESWIDTH,NTRACES) initializes a stripchart in the
% current axes with NTRACES traces.  A trace is a single line on the scope;
% typically one will display one trace per channel of data.  The default
% for NTRACES is 1. To update a stripchart with multiple traces,
% STRIPCHART(S) must specify a matrix S with shorter dimension length =
% NTRACES.  
%
% STRIPCHART(HAX, ...) defines the scope in specified axes HAX instead of GCA.  i.e.,
% STRIPCHART(HAX,FS,AXESWIDTH) initializes axes HAX as a stripchart, and
% STRIPCHART(HAX,S) updates axes HAX with vector S.
%
% HAX = STRIPCHART(...) returns a handle to the axes initialized by the
% stripchart.  This is useful if you allow STRIPCHART to create an
% axes for you, and want to be able to easily reference the axes for
% updates.  The lines created by STRIPCHART all have the tag
% 'StripChart'.  If you would like to manually modify the properties of
% these lines, their handles can be found by:
%
%        HAX = STRIPCHART(...);
%        HLINE = findobj(HAX,'Tag','StripChart');
%
% Example
% 		%% Create data
%       Fs = 1000;              % Sample rate
%       Ns = Fs*3;              % Make 5 seconds worth of data
%       t = (0:1:Ns-1)'/Fs;
%       A = sqrt(t);
%       A(1:Ns/2) = A(1:Ns/2);
%       A(end:-1:Ns/2+1) = A(1:Ns/2);
%       s = A.*sin(2*pi*t*1);
% 		
% 		%% Initialize scope
%       clf
%       AxesWidth = 2;          % Axes Width (s)
% 		stripchart(Fs,AxesWidth);
% 		
% 		%% Update scope
%       N = 50;
%       ind = 1:N:Ns;
% 
%       for ii = ind
%           stripchart(s(ii:ii+N-1,:));
%           drawnow;pause(.05);
%       end;

% Original version of StripChart written by Bob Bemis
% This modification was developed by Scott Hirsch to match syntax
%  of SpectrumScope


%% Parse input arguments
% Decision tree:
% + Initialize or update?
%   o If update -> OK
%   o If initialize -> Axes specified, or use GCA?

error(nargchk(1,4,nargin))

%% Initialize or update?
% If first or second input argument is not a scalar, it must be data - i.e. we are
% updating

if numel(varargin{1}) > 1 || numel(varargin{2}) > 1 % Update
    action = 'update';
    
    if nargin==1                % Use current axes
        hAxes = gca;
        data = varargin{1};
    else
        hAxes = varargin{1};    % Axes was specified
        data = varargin{2};
    end;
    
    % If the user has not initialized this scope, do it for them
    parms = getappdata(hAxes,'StripChartParameters');
    
    % Ensure that scope has been initialized
    if isempty(parms)
        % Use default values
        Fs = 1;
        data = rowmajor(data);
        [Ns,NTraces] = size(data);
        AxesWidth = Ns*Fs;
        feval(mfilename,hAxes,Fs,AxesWidth,NTraces);       % This recursive call will initialize the scope
        % Get the new parameter structure
        parms = getappdata(hAxes,'StripChartParameters');
    end;
    
    
    
else                                    % Initialize  
    action = 'init';
    
    if ~isaxes(varargin{1})             % Easy mode, no handle passed in
        % Use current axes
        hAxes = gca;
        Fs = varargin{1};
        AxesWidth = varargin{2};
        Ns = AxesWidth*Fs;              % Number of samples across axes
        if nargin==3
            NTraces = varargin{3};
        else
            NTraces = 1;
        end;
        
    else                                % Expert mode, passed handle in
        hAxes = varargin{1};
        Fs = varargin{2};
        AxesWidth = varargin{3};
        Ns = AxesWidth*Fs;              % Number of samples across axes
        if nargin==4
            NTraces = varargin{4};
        else
            NTraces = 1;
        end;
    end;
end;


%% Dole out the work
% 
switch action
    case 'init'     % Initialize

        % Build structure to internally pass information
        parms.Fs = Fs;                      % Sample Rate
        parms.NTraces = NTraces;            % Number of lines in plot
        parms.hAxes = hAxes;                % Handle to axes
        parms.Ns = Ns;                      % Number of samples across axes
        parms.AxesWidth = AxesWidth;        % Requested Axes Width (s)
        
        % Store parameter structure
        setappdata(hAxes,'StripChartParameters',parms);
        
        localInitScope(parms)               % Initialize scope
        
    case 'update'   % Update
        parms = getappdata(hAxes,'StripChartParameters');

        % Error checking
        % Ensure that scope has been initialized.  This shouldn't slip
        % through to here.
        if isempty(parms)
            error(['The spectrum scope must first be initialized ' ...
                    'with the sample rate: stripchart(hAxes,Fs)']);
        end;
        
        % Force data to be in columns.  Allow for multiple columns.  This will
        % error if data actually has more channels than samples.
        data = rowmajor(data);

        % Check that the number of columns corresponds to the number of lines
        nc = size(data,2);      % Number of columns
        if nc ~= parms.NTraces
            error(['Size mismatch.  You initialized stripchart with ' num2str(parms.NTraces) ...
                    ' lines, but just passed in ' num2str(nc) ' channels of data.  These' ...
                    ' numbers must be the same.']);
        end;
        
        localUpdateScope(data,parms)            % Update the scope
end;

% Return appropriate output argument
if nargout
    varargout{1} = parms.hAxes;
end;        
            

% ***********************************************************************  
% Initialize the Scope
function localInitScope(parms)

% Set axes
t = (0:1:parms.Ns-1)'/parms.Fs;

% Add line(s)
parms.hLine = plot(t,NaN*ones(length(t),parms.NTraces), ...
    'Tag','StripChart', ...
    'Parent',parms.hAxes);
set(parms.hAxes,'XLim',[0 parms.AxesWidth]);
setappdata(parms.hAxes,'StripChartParameters',parms);

%% Get handle to the figure
% Turn doublebuffer on to eliminate flickering
hFig = get(parms.hAxes,'Parent');

% In R14, it's possible that hFig would return a handle to a panel, not a
% figure
if ~strcmp(get(hFig,'Type'),'figure')
    hFig = get(hFig,'Parent');
end;

%%
% Format the axes 
% maintain X tick spacing
Ticks = get(parms.hAxes,'XTick');
dX = mean(diff(Ticks));

% enforce right justification for grid lines & remove X tick labels
Range = get(parms.hAxes,'XLim');
if Range(end)~=Ticks(end)
  Ticks = fliplr(Range(end):-dX:Range(1));
  set(parms.hAxes,'XTick',Ticks)
end
set(parms.hAxes,'XTickLabel',[],'XTickMode','Manual','XGrid','On','YGrid','On')

%%
% Label the plot.
% There's a bug in R13 when creating xlabel and ylabel with direct
% parenting - the alignment gets all messed up. Instead, make hAx current
% axes
ca = gca;
set(hFig,'CurrentAxes',parms.hAxes);
xlabel(sprintf('%g%s/div',dX,'s'))
ylabel('Amplitude');
set(hFig,'CurrentAxes',ca);

axis manual


%%
% Turn doublebuffer on to eliminate flickering
set(hFig,'DoubleBuffer','on');

% ***********************************************************************  
% Update the plot.
function localUpdateScope(data,parms)
% Callback function to update stripchart with new data

% Dynamically modify Magnitude axis as we go.  Expand, but don't shrink.  
maxM=max(data(:));
minM=min(data(:));
yax2=get(parms.hAxes,'YLim');
if minM<yax2(1),
   yax2(1)=minM;
end
if maxM>yax2(2),
   yax2(2)=maxM;
end
set(parms.hAxes,'YLim',yax2)


hLine = parms.hLine;

[newPts,NLines] = size(data);
yData = get(hLine,'YData');                     % old data

if NLines==1         %Special case for one line only
    yData(1:end-newPts) = yData(newPts+1:end);      % shift old data left
    yData(end-newPts+1:end) = data;                 % new data goes on right
    set(hLine,'YData',yData)                        % update plot
else
    for ii=1:NLines
        yData{ii}(1:end-newPts) = yData{ii}(newPts+1:end);      % shift old data left
        yData{ii}(end-newPts+1:end) = data(:,ii);              % new data goes on right
    end;
    set(hLine,{'YData'},yData)                        % update plot
end;

% ***********************************************************************  
% Utility - isaxes
function truefalse = isaxes(h)
% ISAXES(H)  True if H is a handle to a valid axes

truefalse = 0;      % Start false
if ishandle(h)
    if strcmp('axes',get(h,'Type'))
        truefalse = 1;
    end;
end;

% ***********************************************************************  
% Utility - rowmajor
function data = rowmajor(data)
% Force data to be row major. i.e. more rows than columns

[nr,nc] = size(data);
if nc>nr
    data = data';
end;



