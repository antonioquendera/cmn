load ('lab 11-03-25\dead_zone.mat')

figure;
plot(dead_zone(:,1),dead_zone(:,3), 'LineWidth', 1.5);
hold on
plot(dead_zone(:,1),dead_zone(:,2), 'LineWidth', 1.5);
hold off
xlabel('Time (s)');
ylabel('Angle (rads)', 'Interpreter', 'latex');
legend('$\alpha$','$\beta$' ,'Interpreter', 'latex');
title('Dead-zone $\alpha$ and $\beta$ plot', 'Interpreter', 'latex');
grid on;

% subplot(2,1,2);
% plot(dead_zone(:,1),dead_zone(:,2), 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('$\alpha$ (rads)','Interpreter','latex');
% legend('dead-zone');
% title('Dead-zone $\alpha$ plot','Interpreter','latex');
% grid on;
