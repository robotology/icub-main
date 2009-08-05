%I think this is the most up-to-date script to generate the initial ball points.
%generate points along two circumferences at 0.9 and 1.1 the radius of the actual ball.

R=198/(2*pi) %ball radius.
R1=(1-0.20)*R; %ray for placing the internal points [mm]
R2=(1+0.20)*R; %ray for placing the external points [mm]
nPoints=50;

t=0:(2*pi/nPoints):2*pi;
z=cos(t(1:nPoints));
y=sin(t(1:nPoints));
p1=R1*[zeros(1,nPoints);y;z];
p2=R2*[zeros(1,nPoints);y;z];
points=[p1,p2];

ID=fopen('initial_ball_points_smiley_31mm_20percent.csv','w');
for(a=1:3)
  for(b=1:nPoints*2)
       fprintf(ID,'%f\n',points(a,b));
  end;
end;    
fclose(ID);
