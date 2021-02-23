function err = validate_experiment(csv_filename)
% load the data from the csv 
[x,u] = load_csv(csv_filename);

% Initialize the time stp
timeStep = 0.05;

% create an array of zeros to store time steps and state vector
t=zeros(length(x),1);
xe = {};
for i=1:length(x)
    t(i) = (i-1)*timeStep;
    xe{i} = simulate_bicycle_euler(x(:,i),u(:,i),timeStep);
end
xe =cell2mat(xe);
% plot the results


err = immse(x,xe);



%change to the desired value     
fig = figure();
set(gcf,'color','w');
set(gcf, 'Position',  [100, 100, 1600, 1200])




subplot(2,2,[1,2]);
plot(t(1,1),x(4,1),'*','Color', [38, 38, 38]/255,'DisplayName','init','LineWidth',2);
hold on;
plot(t,x(4,:),'-','Color', [157, 158, 157]/255,'DisplayName','ground-truth,gazebo','LineWidth',2)
plot(t,xe(4,:),'--','DisplayName','sys-id model','Color', [70, 143, 199]/255,'LineWidth',2)
xlabel('t (seconds)') 
ylabel('Vehicle heading (radians)') 
ax = gca; % Get handle to current axes.
ax.XColor = [87, 93, 97]/255; % Red
ax.YColor = [87, 93, 97]/255; % Blue
set(gca,'box','off');
t= title("Vehicle heading",'Color',[87, 93, 97]/255);
set(t, 'horizontalAlignment', 'left')
set(t, 'units', 'normalized')
set(t, 'position', [0.01 1.01 0]);
legend('init','ground-truth','sys-id model','Location','northwest')
legend boxoff 


subplot(2,2,[3,4]);
plot(x(1,1),x(2,1),'*','Color', [38, 38, 38]/255,'DisplayName','init','LineWidth',2);
hold on 
plot(x(1,:),x(2,:),'-','Color', [157, 158, 157]/255,'DisplayName','ground-truth','LineWidth',2) ;
plot(xe(1,:),xe(2,:),'--','DisplayName','prediction','Color', [70, 143, 199]/255,'LineWidth',2)





hold off;
xlabel('x (meters)') 
ylabel('y (meters)') 
t= title("Vehicle Position (map frame)",'Color',[87, 93, 97]/255);
set(t, 'horizontalAlignment', 'left')
set(t, 'units', 'normalized')
set(t, 'position', [0.01 1.01 0]);

legend('init','ground-truth','sys-id','Location','northwest')
legend boxoff 
ax = gca; % Get handle to current axes.
ax.XColor = [87, 93, 97]/255; % Red
ax.YColor = [87, 93, 97]/255; % Blue
set(gca,'box','off');

set(findobj(gcf,'type','axes'),'FontName','Calibri','FontSize',11,'FontWeight','Bold', 'LineWidth', 2,'layer','top');
sgt =sgtitle(strcat('Validation MSE=',string(err)));
sgt.FontSize = 20;
figname = split(strrep(csv_filename,'csv/',''),".");
savename = strcat("plots/",figname(1),".png");
saveas(fig,savename);

fig = figure();
set(gcf,'color','w');
%set(gcf, 'Position',  [100, 100, 1200, 900])
plot(x(1,1),x(2,1),'*','Color', [38, 38, 38]/255,'DisplayName','init','LineWidth',2);
hold on 
plot(x(1,:),x(2,:),'-','Color', [157, 158, 157]/255,'DisplayName','ground-truth','LineWidth',2) ;
plot(xe(1,:),xe(2,:),'--','DisplayName','prediction','Color', [70, 143, 199]/255,'LineWidth',2)

hold off;
xlabel('x (meters)') 
ylabel('y (meters)') 
t= title(strcat('Vehicle Position (map frame), MSE=',string(err)),'Color',[87, 93, 97]/255);
set(t, 'horizontalAlignment', 'left')
set(t, 'units', 'normalized')
set(t, 'position', [0.01 1.01 0]);

legend('init','ground-truth','sys-id','Location','northwest')
legend boxoff 
ax = gca; % Get handle to current axes.
ax.XColor = [87, 93, 97]/255; % Red
ax.YColor = [87, 93, 97]/255; % Blue
set(gca,'box','off');
set(findobj(gcf,'type','axes'),'FontName','Calibri','FontSize',11,'FontWeight','Bold', 'LineWidth', 2,'layer','top');




end

