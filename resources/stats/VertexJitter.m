raw_jitter = csvread('vertex_jitter.csv',0);
p1 = raw_jitter(:,1:2);
p2 = raw_jitter(:,3:4);
p3 = raw_jitter(:,5:6);
p4 = raw_jitter(:,7:8);
stds = [std(p1); std(p2); std(p3); std(p4)];
mns = mean(stds);
norm(mns)
max(norm(stds))