function o = show_seq(num)

stest = sprintf("tests/test%02d.txt",num);
otest = sprintf("output/result_test%02d.txt",num);

s = load(stest);
[h w] = size(s);
if h>1 
  s = s';
endif;
o = load(otest);

#o = o(1:(200-length(s)-length(o)));

del = (max([s o])-min([s o]))*0.1;


if (length(s)>100)
  freq = 20;
else
  freq = 1;
endif;
so = [s o];
sdom = 1:freq:length(s);
sodom = 1:freq:length(so);

axis([1 length(s)+length(o) min([s o])-del max([s o])+del]);
plot(sodom,so(sodom),'r');
hold on;
plot(sdom,s(sdom),'b*-');
hold off;
axis('off');


print(sprintf("/tmp/report_test%02d.eps",num),"-color");

