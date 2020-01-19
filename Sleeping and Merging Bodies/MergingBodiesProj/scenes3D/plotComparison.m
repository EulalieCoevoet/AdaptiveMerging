clear 
close all

X_unmerged = readtable("tower30.csv")
X_merged = readtable("tower30_merged.csv")

num_timesteps = min(height(X_unmerged), height(X_merged))

hfig = figure('Renderer', 'painters', 'Position', [10 10 600 400]), set(gcf,'color','w'); hold on;

col = [ 
    1,  0.4,  0.0 
    1,  0.2, 0.0
    0,  0.4, 1
    0,  0.2, 1
    ];

hold;
yyaxis left
%plot 20th column... total compute_time.
semilogy(X_merged{1:num_timesteps, 20 }, 'DisplayName','Merging')
hold;
semilogy(X_unmerged{1:num_timesteps, 20}, 'DisplayName','No Merging')
hold;
ylabel("Computation Time (s)")

yyaxis right
plot(X_merged{1:num_timesteps, 2}, 'DisplayName','Merging')
hold;
plot(X_unmerged{1:num_timesteps, 2}, 'DisplayName','No Merging')
ylabel("# Contacts")

hold off

legend

xlabel("Simulation Timestep (0.05s)")


fontSize = 10;
fontName = 'Times New Roman';

set(gca, 'fontsize', fontSize);
set(hfig,'Units','Inches');
pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(hfig,'PreliminaryGraph','-dpdf','-r0')
