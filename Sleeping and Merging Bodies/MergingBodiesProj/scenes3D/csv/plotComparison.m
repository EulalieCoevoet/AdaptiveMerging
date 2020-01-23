clear 
close all
scene_name = "funnel1"
plot_name = "Performance With Time for " + scene_name + " Scene"
X_unmerged = readtable(scene_name + ".csv");
X_merged = readtable(scene_name + "_merged.csv");

num_timesteps = min(height(X_unmerged), height(X_merged));

hfig = figure('Renderer', 'painters', 'Position', [10 10 600 400]), set(gcf,'color','w'); hold on;


hold;

h = zeros(4,1);


temp_m = X_merged{1:num_timesteps, 20 } - X_merged{1:num_timesteps, 3};
temp_um = X_unmerged{1:num_timesteps, 20 } - X_unmerged{1:num_timesteps, 3};

subplot(2,1,1);
hold on;

h(1) = plot(temp_um, 'DisplayName','Solve no Merging')

h(3) = plot(temp_m, 'color',[0.8500 0.3250 0.0980], 'DisplayName','Solve Merging')

h(2) =plot(X_unmerged{1:num_timesteps, 3},'color',[0.4940 0.1840 0.5560], 'DisplayName','CD no Merging')



h(4) = plot(X_merged{1:num_timesteps, 3}, 'color',[0.9290 0.6940 0.1250], 'DisplayName','CD Merging')



ylabel("Computation Time (s)")
xlabel("Simulation Timestep")
set(gca,'yscale','log')
legend(h, 'Location', 'southeast')
hold off

subplot(2,1,2);


hold on


plot(X_unmerged{1:num_timesteps, 2}, 'DisplayName','Contacts no Merging')

plot(X_unmerged{1:num_timesteps, 1}, 'color',[0.4940 0.1840 0.5560],  'DisplayName','Bodies no Merging')

plot(X_merged{1:num_timesteps, 2},  'color',[0.8500 0.3250 0.0980], 'DisplayName','Contacts Merging')

plot(X_merged{1:num_timesteps, 1}, 'color',[0.9290 0.6940 0.1250], 'DisplayName','Bodies Merging')




set(gca,'yscale','log')


legend('Location', 'southeast')

ylabel("Number")
xlabel("Simulation Timestep")


fontSize = 10;
fontName = 'Times New Roman';

set(gca, 'fontsize', fontSize);
set(hfig,'Units','Inches');
pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(hfig,'PreliminaryGraph','-dpdf','-r0')
