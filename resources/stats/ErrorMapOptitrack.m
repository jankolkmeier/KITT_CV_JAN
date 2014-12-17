close all;
raw_test = csvread('optitrack_05_macbook.csv',1);
raw_truth = csvread('optitrack_truth.csv');
raw_test_match = raw_test(raw_test(:,3)==1,:);
raw_truth_match = raw_truth(raw_test(raw_test(:,3)==1,2)+1,:);

test_close = raw_test_match((raw_test_match(:,6) < 200), [2, 4:6]);
truth_close = raw_truth(test_close(:,1)+1,:);
norm(mean(abs(test_close(:,2:4) - truth_close(:,2:4))))

err = sqrt(sum(abs(raw_test_match(:,4:6) - raw_truth_match(:,2:4)).^2,2));
data = [ raw_truth_match(:,2), raw_truth_match(:,4), min(err, 60)];

x=linspace(min(data(:,1)),max(data(:,1)), 200);
y=linspace(min(data(:,2)),max(data(:,2)), 200);
[X,Y]=meshgrid(x,y);
F=TriScatteredInterp(data(:,1),data(:,2),data(:,3)-1);

figure(1);
colormap('cool');
contourf(X,Y,F(X,Y),100,'LineColor','none');
zlim([0, 100]);
colorbar;
axis equal;
xlim([-200, 200]);
ylim([80, 350]);
hold on;
%scatter(raw_truth_match(:,2), raw_truth_match(:,4),15,jet(length(raw_truth_match)));
plot(raw_truth_match(:,2), raw_truth_match(:,4),'k*');

xlabel('Distance from marker (cm)') % x-axis label
ylabel('Distance from marker (cm)') % y-axis label
cleanfigure;
matlab2tikz('position_mesh.tikz', 'height', '\figureheight', 'width', '\figurewidth', 'showInfo', false);


figure(2);
axis equal;
xlim([-200, 200]);
ylim([80, 350]);
hold on;

plot(raw_truth(:,2), raw_truth(:,4),'-d','Color',[0.05,0.35,0.25]);
plot(raw_test_match(:,4), raw_test_match(:,6),'k*');

for m=1:length(raw_truth_match),
    plot([raw_truth_match(m,2); raw_test_match(m,4)], [raw_truth_match(m,4); raw_test_match(m,6)]);
end
xlabel('Distance from marker (cm)') % x-axis label
ylabel('Distance from marker (cm)') % y-axis label
cleanfigure;
matlab2tikz('position_path.tikz', 'height', '\figureheight', 'width', '\figurewidth', 'showInfo', false);


%figure(3);
%hold on;
%plot(raw_truth(:,2),zeros(length(raw_truth)));
%plot(raw_truth_match(:,2),log(err),'r*');