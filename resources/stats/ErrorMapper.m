raw = csvread('results1.csv');
err = sqrt(sum(abs(raw(:,1:3) - raw(:,4:6)).^2,2));
data = [ raw(:,1), raw(:,3), min(err, 100) ];

x=linspace(min(data(:,1)),max(data(:,1)), 200);
y=linspace(min(data(:,2)),max(data(:,2)), 200);
[X,Y]=meshgrid(x,y);
F=TriScatteredInterp(data(:,1),data(:,2),data(:,3)-1);

figure(4);
colormap('cool');
contourf(X,Y,F(X,Y),100,'LineColor','none');
zlim([0, 100]);
colorbar;
axis equal;
xlim([-150, 150]);
ylim([80, 300]);