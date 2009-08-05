
function result = panoplot(x,y)

# this is octave code

figure(1);
clf;
clearplot;
scatter(x,y);
model = polyfit(x,y,1);
ypred = model(1)*x+model(2);
hold on;
plot(x,ypred)
hold off;

delta = y-ypred;
th = std(delta);
mask = abs(delta)<th*2;
dom = find(mask);

disp([size(dom) size(x)])

figure(2);
clf;
clearplot;
x2 = x(dom);
y2 = y(dom);
scatter(x2,y2);
model2 = polyfit(x2,y2,1);
ypred2 = model2(1)*x2+model2(2);
hold on;
plot(x2,ypred2)
hold off;

result = model2;
