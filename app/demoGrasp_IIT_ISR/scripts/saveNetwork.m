function saveNetwork(net)
% This function save the file 'network.ini' containing the network's data
% in the proper format required by the ctrlLib framework.

fid=fopen('network.ini','w');

szIW=size(net.IW{1});
szLW=size(net.LW{2,1});

fprintf(fid,'\n');
fprintf(fid,'numInputs\t\t\t%d\n',uint8(szIW(2)));
fprintf(fid,'numHiddenNodes\t\t%d\n',uint8(szIW(1)));
fprintf(fid,'numOutputNodes\t\t%d\n',uint8(szLW(1)));
fprintf(fid,'\n\n');

fprintf(fid,'// hidden layer neurons\n');
for i=1:szIW(1)
    fprintf(fid,'IW_%d\t\t( ',uint8(i-1));
    for j=1:szIW(2)
        fprintf(fid,'%f ',net.IW{1}(i,j));
    end
    fprintf(fid,')\n');
end
fprintf(fid,'\n');

fprintf(fid,'// output layer neurons\n');
for i=1:szLW(1)
    fprintf(fid,'LW_%d\t\t( ',uint8(i-1));
    for j=1:szLW(2)
        fprintf(fid,'%f ',net.LW{2,1}(i,j));
    end
    fprintf(fid,')\n');
end
fprintf(fid,'\n');

fprintf(fid,'// bias hidden layer neurons\n');
fprintf(fid,'b1\t\t( ');
for i=1:length(net.b{1})
    fprintf(fid,'%f ',net.b{1}(i));
end
fprintf(fid,')\n\n');

fprintf(fid,'// bias output layer neurons\n');
fprintf(fid,'b2\t\t( ');
for i=1:length(net.b{2})
    fprintf(fid,'%f ',net.b{2}(i));
end
fprintf(fid,')\n\n');

fprintf(fid,'// input preprocessing\n');
params=net.inputs{1}.processSettings{3};
for i=1:szIW(2)
    fprintf(fid,'inMinMaxX_%d\t\t( %f %f )\n',uint8(i-1),params.xmin(i),params.xmax(i));
end
for i=1:szIW(2)
    fprintf(fid,'inMinMaxY_%d\t\t( %f %f )\n',uint8(i-1),params.ymin,params.ymax);
end
fprintf(fid,'\n');

fprintf(fid,'// output preprocessing\n');
params=net.outputs{2}.processSettings{2};
for i=1:szLW(1)
    fprintf(fid,'inMinMaxY_%d\t\t( %f %f )\n',uint8(i-1),params.ymin,params.ymax);
end
for i=1:szLW(1)
    fprintf(fid,'inMinMaxX_%d\t\t( %f %f )\n',uint8(i-1),params.xmin(i),params.xmax(i));
end
fprintf(fid,'\n');

fclose(fid);



