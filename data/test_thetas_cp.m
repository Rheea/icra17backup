%% load files with results
close all
clear all
rearing =1;

if rearing==1
    load('rear_60_hyq2max_good1.mat');
else
    load('mu6_SR_data_terrain_init_25-Aug-2016-12-1.mat');
end

thetas = out.datx(end,:);
timep=0:0.001:3;

green=[0; 0.5; 0];
figure()
subplot(1,2,1)
hold on
grid on
plot(timep, X(1,:),'r','LineWidth',3)
plot(timep, X(2,:),'Color',green,'LineWidth',3)
plot(timep, X(3,:),'b','LineWidth',3)
legend('X axis', 'Y axis', 'Z axis')
xlabel('time (s)')
ylabel('torso position (m)')
axis tight
subplot(1,2,2)
hold on
grid on
plot(timep, RPY(1,:),'r','LineWidth',3)
plot(timep, RPY(2,:),'Color',green,'LineWidth',3)
plot(timep, RPY(3,:),'b','LineWidth',3)
legend('roll', 'pitch','yaw','FontSize',14)
xlabel('time (s)')
ylabel('orientation (rad)')
axis tight

figure()
plot(out.dat(:,1),'LineWidth',3);
hold on
grid on
xlabel('evaluations','fontsize',12);
ylabel('cost','fontsize',12);
% set(gca,'YTick',[])
axis tight
grid minor
% exportfig(gcf,'cost_cma.eps','Color','rgb');

tf=3;
mus = 0:0.2:tf;
sigmas = 0.1*ones(size(mus));

if rearing==1

for tno=1:6
    weights = out.datx(end,1+(tno-1)*16:tno*16);                    

    figure(333)
    x = 0:0.005:tf;
    y = zeros(1,size(x,2));
    for i=1:size(x,2);
        y(i) = gaussianWeightedRegression(x(i),mus,sigmas,weights);
    end
    hold on
    grid on
%     subplot(3,2,tno)
    switch tno
        case 1
            subplot(3,2,1)
        case 2
            subplot(3,2,3)
        case 3
            subplot(3,2,5)
        case 4
            subplot(3,2,2)
        case 5
            subplot(3,2,4)
        case 6
            subplot(3,2,6)
    end
    
    switch tno
        case 1
            
        case 2
            y=-y+pi/2;
        case 3
            y=-y-pi;
        case 4
            y=-y;
        case 5
            y=-y-pi/2;
        case 6
            y=-y+pi;
   end   
    
    plot(x,y,'k-','LineWidth',2); hold on; grid on;
    grid minor
    yy = zeros(size(mus,2),size(x,2));
    for i=1:size(mus,2);
        yy(i,:)=weights(i) * gaussmf(x,[sigmas(i) mus(i)]);
    end
%     plot(mus,zeros(size(mus)),'.r','MarkerSize',10);
%     plot(x,yy,'--','LineWidth',1,'Color',[0.8, 0.8, 0.8]);
    
%     legend('Policy prediction','Gaussian centers','Single kernel predictions');
%     chH = get(gca,'Children');
%     set(gca,'Children',[chH(end);chH(end-1);chH(1:end-2)]);
%     hold off;
    xlabel('time (s)')
    
    switch tno
        case 1
            ylabel('Front HAA (rad)')
        case 2
            ylabel('Front HFE (rad)')
        case 3
            ylabel('Front KFE (rad)')
        case 4
            ylabel('Hind HAA (rad)')
        case 5
            ylabel('Hind HFE (rad)')
        case 6
            ylabel('Hind KFE (rad)')
    end
    
    axis tight
    % title('regurlarly spaced \mu `s, \sigma^2 = 0.01 and randomized weights ')
    legend('boxoff');
%    exportfig(gcf,'Gaussian_policy.eps','Color','rgb');
end


% fprintf('The cost Task init is %f.\n',out.dat(end,1));
% % index=find(out.dat(:,1)==min(out.dat(:,1)))
% hyq_do_Rollout(out.datx(end,:),1)
else
%% Pose Rec
%%%% for 12 DOf X 16 Gaussians (Pose Recovery)
    
    for tno=1:6
    weights = out.datx(end,1+(tno-1)*16:tno*16);                    

    figure(111)
    x = 0:0.005:tf;
    y = zeros(1,size(x,2));
    y2 = zeros(1,size(x,2));
    for i=1:size(x,2);
        y(i) = gaussianWeightedRegression(x(i),mus,sigmas,weights);       
    end

    hold on
    grid on
    switch tno
        case 1
            subplot(3,2,1)
        case 2
            subplot(3,2,3)
        case 3
            subplot(3,2,5)
        case 4
            subplot(3,2,2)
        case 5
            subplot(3,2,4)
        case 6
            subplot(3,2,6)
    end
    
    
      switch tno
        case 1
            
        case 2
            y=-y+pi/2;
        case 3
            y=-y-pi;
        case 4
            
        case 5
            y=-y+pi/2;
        case 6
            y=-y-pi;
      end   
    
    plot(x,y,'k-','LineWidth',2); hold on; grid on;
%         if tno==2
%             plot(x,y2,'m-','LineWidth',2); 
%         else
%         end
    yy = zeros(size(mus,2),size(x,2));
%     yy2 = zeros(size(mus,2),size(x,2));

    for i=1:size(mus,2);
        yy(i,:)=weights(i) * gaussmf(x,[sigmas(i) mus(i)]);
%         if tno==2
%             yy2(i,:) = weights(i) * gaussmf(x,[sigmas(i) -mus(i)+pi/2]);
%         else
%         end
    end
%     plot(mus,zeros(size(mus)),'.r','MarkerSize',10);
%     plot(x,yy,'--','LineWidth',1,'Color',[0.8, 0.8, 0.8]);
% %     legend('Policy prediction','Gaussian centers','Single kernel predictions');
%     chH = get(gca,'Children');
%     set(gca,'Children',[chH(end);chH(end-1);chH(1:end-2)]);
%     hold off;
    xlabel('time (s)')
    
%     switch tno
%         case 1
%             ylabel('LF HAA (rad)')
%         case 2
%             ylabel('RF HAA (rad)')
%         case 3
%             ylabel('LF HFE (rad)')
%         case 4
%             ylabel('RF HFE (rad)')
%         case 5
%             ylabel('LF KFE (rad)')
%         case 6
%             ylabel('RF KFE (rad)')
%     end
    
     switch tno
        case 1
            ylabel('LF HAA (rad)')
        case 2
            ylabel('LF HFE (rad)')
        case 3
            ylabel('LF KFE (rad)')
        case 4
            ylabel('RF HAA (rad)')
        case 5
            ylabel('RF HFE (rad)')
        case 6
            ylabel('RF KFE (rad)')
    end   
    axis tight
    grid minor
    % title('regurlarly spaced \mu `s, \sigma^2 = 0.01 and randomized weights ')
%     legend('boxoff');
%    exportfig(gcf,'Gaussian_policy.eps','Color','rgb');
    end
% end


   for tno=7:12
    weights = out.datx(end,1+(tno-1)*16:tno*16);                    

    figure(222)
    x = 0:0.005:tf;
    y = zeros(1,size(x,2));
    for i=1:size(x,2);
        y(i) = gaussianWeightedRegression(x(i),mus,sigmas,weights);
    end
    hold on
    grid on
    switch tno
        case 7
            subplot(3,2,1)
        case 8
            subplot(3,2,3)
        case 9
            subplot(3,2,5)
        case 10
            subplot(3,2,2)
        case 11
            subplot(3,2,4)
        case 12
            subplot(3,2,6)
    end
    
    switch tno
        case 7
            y=-y;
        case 8
            y=-y-pi/2;
        case 9
            y=-y+pi;
        case 10
            y=-y;
        case 11
            y=-y-pi/2;
        case 12
            y=-y+pi;
    end   
    
    plot(x,y,'k-','LineWidth',2); hold on; grid on;

    yy = zeros(size(mus,2),size(x,2));
    for i=1:size(mus,2);
        yy(i,:)=weights(i) * gaussmf(x,[sigmas(i) mus(i)]);
    end
%     plot(mus,zeros(size(mus)),'.r','MarkerSize',10);
%     plot(x,yy,'--','LineWidth',1,'Color',[0.8, 0.8, 0.8]);
    
%     legend('Policy prediction','Gaussian centers','Single kernel predictions');
%     chH = get(gca,'Children');
%     set(gca,'Children',[chH(end);chH(end-1);chH(1:end-2)]);
% %     hold off;
    xlabel('time (s)')
    
%     switch tno
%         case 7
%             ylabel('LH HAA (rad)')
%         case 8
%             ylabel('RH HAA (rad)')
%         case 9
%             ylabel('LH HFE (rad)')
%         case 10
%             ylabel('RH HFE (rad)')
%         case 11
%             ylabel('LH KFE (rad)')
%         case 12
%             ylabel('RH KFE (rad)')
%     end
    
   switch tno
        case 7
            ylabel('LH HAA (rad)')
        case 8
            ylabel('LH HFE (rad)')
        case 9
            ylabel('LH KFE (rad)')
        case 10
            ylabel('RH HAA (rad)')
        case 11
            ylabel('RH HFE (rad)')
        case 12
            ylabel('RH KFE (rad)')
   end 
    axis tight
    grid minor
    
    % title('regurlarly spaced \mu `s, \sigma^2 = 0.01 and randomized weights ')
%     legend('boxoff');
%    exportfig(gcf,'Gaussian_policy.eps','Color','rgb');
   end
    
   
end