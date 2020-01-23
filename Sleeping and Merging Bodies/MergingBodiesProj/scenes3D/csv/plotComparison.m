clear 
close all
scene_name = "funnel1"
plot_name = "Performance With Time for " + scene_name + " Scene"
X_unmerged = readtable(scene_name + ".csv");
X_merged = readtable(scene_name + "_merged.csv");

num_timesteps = min(height(X_unmerged), height(X_merged));

hfig = figure('Renderer', 'painters', 'Position', [10 10 1200 800]), set(gcf,'color','w'); hold on;


hold;

temp_m = X_merged{1:num_timesteps, 20 } - X_merged{1:num_timesteps, 3};
temp_um = X_unmerged{1:num_timesteps, 20 } - X_unmerged{1:num_timesteps, 3};

subplot(2,1,1);
hold on;




plot(temp_m, 'DisplayName','Collision Processing with Merging')

plot(X_merged{1:num_timesteps, 3}, 'DisplayName','Collision Detection with Merging')

plot(temp_um, 'DisplayName','Collision Processing without Merging')

plot(X_unmerged{1:num_timesteps, 3}, 'DisplayName','Collision Detection without Merging')

ylabel("Computation Time (s)")
xlabel("Simulation Timestep")
set(gca,'yscale','log')
legend('Location', 'southeast')
hold off

subplot(2,1,2);


hold on


plot(X_merged{1:num_timesteps, 2},  'DisplayName','# Contacts with Merging')

plot(X_merged{1:num_timesteps, 1},  'DisplayName','# Bodies with Merging')

plot(X_unmerged{1:num_timesteps, 2}, 'DisplayName','# Contacts without Merging')

plot(X_unmerged{1:num_timesteps, 1},   'DisplayName','# Bodies without Merging')

set(gca,'yscale','log')


legend('Location', 'southeast')

ylabel("#")
xlabel("Simulation Timestep")


fontSize = 10;
fontName = 'Times New Roman';

set(gca, 'fontsize', fontSize);
set(hfig,'Units','Inches');
pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(hfig,'PreliminaryGraph','-dpdf','-r0')
