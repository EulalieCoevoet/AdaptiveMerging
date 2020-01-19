fontSize = 10;
fontName = 'Times New Roman';

hfig = figure('Renderer', 'painters', 'Position', [10 10 600 400]), set(gcf,'color','w'); hold on;

% Here is an example of collecting some invisible plots in h to % create a legend... only necessary if you do lots more plotting % than you want to have legend entries...
lambdas = [ 0.5, 0.4533458, 1/3, 0.25 ];
h = zeros(4,1);
for lambda = lambdas
     h(index) = plot(NaN,NaN,'-','Color', col(index,:));
     index = index + 1;
end

title( 'variable reflectivity curves'  ); xlabel('Frequency (Hz)'); ylabel('Amplitude (linear scale)');

xlim([0 fMaxPlot]);
xline( simParamsopt.MaxFrequency, '--r' );

legend( h, ...
     sprintf( 'lambda = %.3g', lambdas(1) ),...
     sprintf( 'lambda = %.3g', lambdas(2) ),...
     sprintf( 'lambda = %.3g', lambdas(3) ),...
     sprintf( 'lambda = %.3g', lambdas(4) ), 'location', 'southwest' );


set(gca, 'fontsize', fontSize, 'fontname', fontName); set(hfig,'Units','Inches'); pos = get(hfig,'Position'); set(hfig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3),
pos(4)])
print(hfig,'figures/figRMagAllLambdaLinear','-dpdf','-r0')