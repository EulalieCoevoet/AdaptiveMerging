clear 
close all
scene_name = "tower30"
plot_name = "Performance With Time for " + scene_name + " Scene"
X_unmerged = readtable(scene_name + ".csv")
X_merged = readtable(scene_name + "_merged.csv")

num_timesteps = min(height(X_unmerged), height(X_merged))

hfig = figure('Renderer', 'painters', 'Position', [10 10 600 400]), set(gcf,'color','w'); hold on;


hold;

subplot(2,1,1);
semilogy(X_merged{1:num_timesteps, 20 }, 'DisplayName','Merging')
hold;
semilogy(X_unmerged{1:num_timesteps, 20}, 'DisplayName','No Merging')
hold;
ylabel("Computation Time (s)")

legend

subplot(2,1,2);

semilogy(X_merged{1:num_timesteps, 2}, 'DisplayName','Merging')
hold;
semilogy(X_unmerged{1:num_timesteps, 2}, 'DisplayName','No Merging')
ylabel("# Contacts")
ylim([10^(1) 10^4])



legend

xlabel("Simulation Timestep")


fontSize = 10;
fontName = 'Times New Roman';

set(gca, 'fontsize', fontSize);
set(hfig,'Units','Inches');
pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(hfig,'PreliminaryGraph','-dpdf','-r0')
