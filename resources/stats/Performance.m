
p03 = csvread('optitrack_03.csv',1);
p05 = csvread('optitrack_05.csv',1);
p07 = csvread('optitrack_07.csv',1);
p10 = csvread('optitrack_10.csv',1);

figure(1); clf; hold on;

f_range = 1:100;
plot(p03(f_range,2), p03(f_range,8),'r',...
     p05(f_range,2), p05(f_range,8),'g',...
     p07(f_range,2), p07(f_range,8), 'b',...
     p10(f_range,2), p10(f_range,8), 'k',...
     p03(p03(f_range, 3)==1, 2), p03(p03(f_range, 3)==1, 8), 'r*');
xlabel('Frame #') % x-axis label
ylabel('Computation time in ms') % y-axis label
legend('scale=0.3','scale=0.5','scale=0.7','scale=1.0', 'Location','northeast')

cleanfigure;
matlab2tikz('time_frame_scale.tikz', 'height', '\figureheight', 'width', '\figurewidth', 'showInfo', false);

mp = [mean(p03(f_range,8)), mean(p05(f_range,8)), mean(p07(f_range,8)), mean(p10(f_range,8))];
cap_w = 1280;
cap_h = 720;
pa = [ (cap_w*0.3) * (cap_h*0.3), (cap_w*0.5) * (cap_h*0.5), (cap_w*0.7) * (cap_h*0.7), (cap_w) * (cap_h) ];

figure(2); clf; hold on;

plot(pa, mp, '-k*');
ax = gca;
xlabel('Resolution of search image') % x-axis label
ylabel('Mean computation time in ms') % y-axis label
ax.XTick = pa;
ax.XTickLabelMode = 'manual';
ax.XTickLabel = {'384x216', '640x360', '896x504', '1280x720'};
cleanfigure;
matlab2tikz('time_resolution.tikz', 'height', '\figureheight', 'width', '\figurewidth', 'showInfo', false);

m_grab = mean(p03(:,7));
m_prep = mean(p03(:,9));
m_search = mean(p03(:,10));
m_approx = mean(p03(:,11));
m_refine = mean(p03(:,12));
m_estimate = mean(p03(:,13));
m_lst = [m_grab; m_prep; m_search; m_approx; m_refine; m_estimate];
total = sum(m_lst);
m_lst/total


%legend('...','Location','northeast')