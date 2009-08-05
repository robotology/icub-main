function [NumberOfSamples, NameOfSignals, Signals] = ReadCollectorTxt(varargin)

%Genova 03/08/2005
%Edited by Francesco Nori
%
% This function reads a .txt file containing
% the data sent by the DSP. The first row of
% the given .txt is assumed to contain the
% names of the read variables. Columns are assumed
% to be separated by spaces. The function can be used as follows:
%
%       ReadJamesTxt(FileName, Delimiter, SampleTime, Flag): collects all data in the
%       file without excluding any column. Data are displayed in a figure
%       if Flag == 1; otherwise no plot is displyed. 
%       Example: ReadJamesTxt('test.txt', ',', 0.01, 1) reads the data in test.txt 
%       assuming that data are separated by comas and sampled at T = 0.01sec
%       A plot of the data is displayed.
%
%       ReadJamesTxt(FileName, Delimiter, SampleTime, Flag, Columns): collects only data
%       corresponding to the columns contained in the vector Columns. 
%       Example: ReadJamesTxt('test.txt', ',', 0.1, 0, [1 3]) reads the data 
%       in test.txt assuming that data are separated by comas and sampled at T=0.1. Only the
%       first and the third columns are considered and not plotted.

if length(varargin) == 4    
    FileName = varargin{1};
    Delimiter = varargin{2};
    Ts = varargin{3};
    Flag = varargin{4};
    
    fid = fopen(FileName);
    FirstLine = fgetl(fid);
    NameOfSignals = strread(FirstLine, '%s','delimiter', Delimiter);
    Columns = 1:length(NameOfSignals);
    fclose(fid);
elseif length(varargin) == 5
    FileName = varargin{1};
    Delimiter = varargin{2};
    Ts = varargin{3};
    Flag = varargin{4};
    Columns = varargin{5};
end

fid = fopen(FileName);
FirstLine = fgetl(fid);
NameOfSignals = strread(FirstLine, '%s','delimiter', Delimiter);
NameOfSignals = NameOfSignals(Columns);

Line = FirstLine;
Signals = [];
NumberOfSamples = 0;
Line = fgetl(fid);
while Line ~=-1
    NumberOfSamples = NumberOfSamples + 1;
    Row = strread(Line, '%f','delimiter', Delimiter);
    Row = Row(Columns);
    Signals = cat(1, Signals, Row');
    Line = fgetl(fid);
end

if Flag == 1
    figure
    hold on
    for i = Columns
        index = find(Columns==i);
        str = strcat(num2str(i), ': ', NameOfSignals(index));
        pos_x = ceil(index*NumberOfSamples/length(Columns));
        text(Ts*(pos_x-1), Signals(pos_x, index), str);
    end
    [m, n] = size(Signals);
    plot((0:m-1).*Ts, Signals)
    %legend(Leg)
    for i = Columns
        index = find(Columns==i);
        NameOfSignals(index) = strcat(num2str(i), ': ', NameOfSignals(index));
    end 
end

fclose(fid);
