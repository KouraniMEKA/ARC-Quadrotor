load('IARC')



t=IARC.time(:,1);
I_ARC=IARC.Data(:,1);
I_IARC=IARC.Data(:,2);
I_Real=IARC.Data(:,3);
I_Max=IARC.Data(:,4);
I_Min=IARC.Data(:,5);

dt=t(2)-t(1); %2ms
time0=0:dt:(length(t)-1)*dt;

close all
%% %%%%%%%%%%%%% Plot Z %%%%%%%%%%%%%
x0=9; 
y0=4; %position of the lower left side of the figure
width=5;
height=4;
figure('Units','inches',...
'Position',[x0 y0 width height],...
'PaperPositionMode','auto');

hold on
plot(time0,I_IARC,'k','LineWidth',1.5)
plot(time0,I_ARC,'b','LineWidth',1)
plot(time0,I_Real,'c:','LineWidth',1.5)
plot(time0,I_Max,'r-.','LineWidth',1)
plot(time0,I_Min,'g-.','LineWidth',1)

xlabel('Time (s)')
ylabel('I (Kg.m^2)')

axis([0 80 0.01 0.05])
title('Moment of inertia estimation')
set(gca,...
'Units','normalized',...
'YTick',0:.005:0.05,...
'XTick',0:10:80,...
'FontUnits','points',...
'FontWeight','normal',...
'FontSize',10,...
'FontName','Times')

legend('I_{IARC}','I_{ARC}','I_{Real}','I_{Max}','I_{Min}',...
'FontUnits','points',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times',...
'Location','SouthEast')

box on
grid on
set(gca,'gridlinestyle','-')

print -depsc2 I_IARC.emf
