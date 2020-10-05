function plot_results(output,model,brain,if_UIO,if_cause)
%% Plot the results

%
color = [ 0         0.4470    0.7410;    0.8500    0.3250    0.0980;
          0.9290    0.6940    0.1250;    0.4940    0.1840    0.5560;
          0.4660    0.6740    0.1880;    0.3010    0.7450    0.9330;
          0.6350    0.0780    0.1840];

% plot DEM with and without causes
if if_cause == 1
    figure;
    fig1 = plot(output.DEM_t,output.DEMv_x(:,1:brain.nx),'-.','Color',color(1,:)); hold on;
    fig2 = plot(output.DEM_t,output.kalmfv_x,'--','Color',color(2,:)); hold on;
    fig3 = plot(output.DEM_t,model.ideal_x,'-','Color',color(4,:));
    legend([fig1(1),fig2(1),fig3(1)],...
        {'DEMv states','KFv states','Ideal states'});
    xlabel time(s);
end

% plot state estimates and true states
figure; subplot(2,1,1);
fig1 = plot(model.t,model.ideal_x,'-','Color',color(1,:)); hold on;
fig2 = plot(output.DEM_t,output.DEM_x(:,1:brain.nx),'--','Color',color(2,:)); hold on;
fig3 = plot(model.t,output.kalman_x','-.','Color',color(4,:)); hold on;
if if_UIO == 1
    fig4 = plot(model.t,output.UIO_x_est,'-.','Color',color(5,:)); hold on;
    legend([fig1(1),fig2(1),fig3(1),fig4(1)],{'Ideal states','DEM states',...
        'Kalman estimate','UIO estimate'});    
else
    legend([fig1(1),fig2(1),fig3(1)],{'Ideal states','DEM states','Kalman estimate'});
end

% plot causal state estimate and true cause
subplot(2,1,2)
plot(output.DEM_t,output.DEM_x(:,brain.nx*(brain.p+1)+1),'--'); hold on;
plot(model.t,model.real_cause','-','LineWidth',1); hold on;
if if_UIO == 1
    plot(model.t,output.UIO_v_est,'-.'); hold on;
    legend('DEM cause estimate','Ideal causal state','UIO cause estimate');
else
    legend('DEM cause estimate','Ideal causal state');    
end

% plot kalman state estimates and true states
% figure; 
% fig1 = plot(output.DEM_t,output.DEM_x(:,1:brain.nx),'--','Color',color(1,:)); hold on;
% fig2 = plot(model.t,output.kalman_x','-.','Color',color(2,:)); hold on;
% fig3 = plot(model.t,model.ideal_x,'-','Color',color(4,:));
% legend([fig1(1),fig2(1),fig3(1)],...
%     {'DEM states','Kalman states','Ideal states'});
% xlabel time(s)
end