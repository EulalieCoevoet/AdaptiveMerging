clear 
close all
scene_name = "funnel1"
plot_name = "Performance With Time for " + scene_name + " Scene"
X_unmerged = readtable(scene_name + ".csv")
X_merged = readtable(scene_name + "_merged.csv")

num_timesteps = min(height(X_unmerged), height(X_merged))

hfig = figure('Renderer', 'painters', 'Position', [10 10 600 400]), set(gcf,'color','w'); hold on;


hold;

temp_m = X_merged{1:num_timesteps, 20 } - X_merged{1:num_timesteps, 3}
temp_um = X_unmerged{1:num_timesteps, 20 } - X_unmerged{1:num_timesteps, 3}
subplot(3,1,1);
plot(temp_m, 'DisplayName','Merging (No CD)')
hold;
plot(temp_um, 'DisplayName','No Merging (No CD)')
hold;
ylabel("Computation Time (s)")

legend


subplot(3,1,2);

semilogy(X_merged{1:num_timesteps, 3}, 'DisplayName','Merging CD')
hold;
semilogy(X_unmerged{1:num_timesteps, 3}, 'DisplayName','No Merging CD')
legend
ylabel("Collision Detection Time")


subplot(3,1,3);

semilogy(X_merged{1:num_timesteps, 2}, 'DisplayName','Merging')
hold;
semilogy(X_unmerged{1:num_timesteps, 2}, 'DisplayName','No Merging')
ylabel("# Contacts")



legend

xlabel("Simulation Timestep")


fontSize = 10;
fontName = 'Times New Roman';

set(gca, 'fontsize', fontSize);
set(hfig,'Units','Inches');
pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(hfig,'PreliminaryGraph','-dpdf','-r0')
