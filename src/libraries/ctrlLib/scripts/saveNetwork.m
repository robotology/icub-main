function saveNetwork(net)
% This function save the file 'network.ini' containing the network's data
% in the proper format required by the ctrlLib framework.

fid=fopen('network.ini','w');

szIW=size(net.IW{1});
szLW=size(net.LW{2,1});

fprintf(fid,'\n');
fprintf(fid,'numInputNodes\t\t%d\n',uint8(szIW(2)));
fprintf(fid,'numHiddenNodes\t\t%d\n',uint8(szIW(1)));
fprintf(fid,'numOutputNodes\t\t%d\n',uint8(szLW(1)));
fprintf(fid,'\n');

fprintf(fid,'// hidden layer neurons\n');
for i=1:szIW(1)
    fprintf(fid,'IW_%d\t\t( ',uint8(i-1));
    for j=1:szIW(2)
        fprintf(fid,'%.15f ',net.IW{1}(i,j));
    end
    fprintf(fid,')\n');
end
fprintf(fid,'\n');

fprintf(fid,'// output layer neurons\n');
for i=1:szLW(1)
    fprintf(fid,'LW_%d\t\t( ',uint8(i-1));
    for j=1:szLW(2)
        fprintf(fid,'%.15f ',net.LW{2,1}(i,j));
    end
    fprintf(fid,')\n');
end
fprintf(fid,'\n');

fprintf(fid,'// bias hidden layer neurons\n');
fprintf(fid,'b1\t\t( ');
for i=1:length(net.b{1})
    fprintf(fid,'%.15f ',net.b{1}(i));
end
fprintf(fid,')\n\n');

fprintf(fid,'// bias output layer neurons\n');
fprintf(fid,'b2\t\t( ');
for i=1:length(net.b{2})
    fprintf(fid,'%.15f ',net.b{2}(i));
end
fprintf(fid,')\n\n');

fprintf(fid,'// input preprocessing\n');
params=net.inputs{1}.processSettings{end};
for i=1:szIW(2)
    fprintf(fid,'inMinMaxX_%d\t\t( %.15f %.15f )\n',uint8(i-1),params.xmin(i),params.xmax(i));
end
for i=1:szIW(2)
    fprintf(fid,'inMinMaxY_%d\t\t( %.15f %.15f )\n',uint8(i-1),params.ymin,params.ymax);
end
fprintf(fid,'\n');

fprintf(fid,'// output postprocessing\n');
params=net.outputs{2}.processSettings{end};
for i=1:szLW(1)
    fprintf(fid,'outMinMaxY_%d\t\t( %.15f %.15f )\n',uint8(i-1),params.ymin,params.ymax);
end
for i=1:szLW(1)
    fprintf(fid,'outMinMaxX_%d\t\t( %.15f %.15f )\n',uint8(i-1),params.xmin(i),params.xmax(i));
end
fprintf(fid,'\n');

fclose(fid);



