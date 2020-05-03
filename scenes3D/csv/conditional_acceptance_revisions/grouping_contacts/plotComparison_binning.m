clear 
close all

% Put name of scene name having 2 csv files in same directory as this file
%(one with name "<scene_name>.csv" and the other "<scene_name>_merged.csv")
scene_name = "funnel"


fontSize = 10;
fontName = 'Times New Roman';

smoothing = 1
plot_name = "Performance With Time for " + scene_name + " Scene"
X_unmerged = readtable(scene_name + ".csv");
X_merged = readtable(scene_name + "_merged.csv");

num_timesteps = 60000 %min(height(X_unmerged), height(X_merged));
partitions = 600
hfig = figure('Renderer', 'painters', 'Position', [10 10 600 400]), set(gcf,'color','w'); hold on;


hold;

h = zeros(4,1);


X_collision_resolution_merged = movmean(X_merged{1:num_timesteps, 13} - X_merged{1:num_timesteps, 3}, smoothing);
X_collision_resolution_unmerged = X_unmerged{1:num_timesteps, 13 } - X_unmerged{1:num_timesteps, 3};
X_collision_detection_merged = X_merged{1:num_timesteps, 3}
X_collision_detection_unmerged = X_unmerged{1:num_timesteps, 3}


X_collision_detection_unmerged = bin(X_collision_detection_unmerged, partitions, num_timesteps)
X_collision_resolution_unmerged = bin(X_collision_resolution_unmerged, partitions, num_timesteps)

X_collision_resolution_merged = bin(X_collision_resolution_merged, partitions, num_timesteps)

X_collision_detection_merged = bin(X_collision_detection_merged, partitions, num_timesteps)






subplot(2,1,1);
hold on;

h(1) = plot(X_collision_resolution_unmerged(:, 1), 'color',[0 0.4470 0.7410], 'LineWidth', 4, 'DisplayName','Col. Resolution no Merging')
plot(X_collision_resolution_unmerged(:, 2), 'color',[0 0.4470 0.7410]) %error bars
plot(X_collision_resolution_unmerged(:, 3), 'color',[0 0.4470 0.7410])

h(3) = plot(X_collision_resolution_merged(:, 1), 'LineWidth', 4,  'color',[0.8500 0.3250 0.0980], 'DisplayName','Col. Resolution w. Merging')
plot(X_collision_resolution_merged(:, 2), 'color',[0.8500 0.3250 0.0980]) %error bars
plot(X_collision_resolution_merged(:, 3), 'color',[0.8500 0.3250 0.0980])

h(2) =plot(X_collision_detection_unmerged(:, 1), 'LineWidth', 4, 'color',[0.4940 0.1840 0.5560], 'DisplayName','Col. Detection no Merging')
plot(X_collision_detection_unmerged(:, 2), 'color',[0.4940 0.1840 0.5560]) %error bars
plot(X_collision_detection_unmerged(:, 3), 'color',[0.4940 0.1840 0.5560])

h(4) = plot(X_collision_detection_merged(:, 1), 'LineWidth', 4, 'color',[0.9290 0.6940 0.1250], 'DisplayName','Col. Detection w. Merging')
plot(X_collision_detection_merged(:, 2), 'color',[0.9290 0.6940 0.1250]) %error bars
plot(X_collision_detection_merged(:, 3), 'color',[0.9290 0.6940 0.1250])

hold;

ylabel("Computation Time (s)")
xlabel("Simulation Timestep")
set(gca,'yscale','log')
set(gca, 'YTick', [10.^-4 10.^-2 10^0 ])

set(gca, 'fontsize', fontSize, 'fontname', fontName);
legend(h, 'Location', 'southeast')
hold off

subplot(2,1,2);



hold on



plot(X_unmerged{1:num_timesteps, 2}, 'DisplayName','Contacts no Merging')

plot(X_unmerged{1:num_timesteps, 1}, 'color',[0.4940 0.1840 0.5560],  'DisplayName','Bodies no Merging')

plot(movmean(X_merged{1:num_timesteps, 2}, smoothing),  'color',[0.8500 0.3250 0.0980], 'DisplayName','Contacts w. Merging')

plot(movmean(X_merged{1:num_timesteps, 1}, smoothing), 'color',[0.9290 0.6940 0.1250], 'DisplayName','Bodies w. Merging')




set(gca,'yscale','log')


legend('Location', 'southeast')

ylabel("Number")
xlabel("Simulation Timestep")



set(gca, 'fontsize', fontSize, 'fontname', fontName);
set(hfig,'Units','Inches');
pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(hfig, scene_name + "_comparison",'-dpdf','-r0')



function binned = bin(X, partitions, num_timesteps)

    partition_len = num_timesteps/partitions;
    %bin data 
    X_mean = zeros(0, 3);
    for i = 0:(partitions-1)
        partition = X(partition_len*i + 1: partition_len*i + partition_len);
        mean_vector = mean(partition) *ones(length(partition), 1);
        max_vector =  max(partition) *ones(length(partition), 1);
        min_vector =  min(partition) *ones(length(partition), 1);
        X_mean = [X_mean;mean_vector max_vector min_vector];
        %X_collision_resolution_mean(partition_len*i: partition_len*i + partition_len-1, 1:3 ) = matrix;
    end
    binned=X_mean;
end
