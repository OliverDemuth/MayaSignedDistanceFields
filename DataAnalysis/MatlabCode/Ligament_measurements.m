% Ligament_measurements.m
%   This script computes measurements from XROMM and ROM ligament 
%   simulation data. Ligament lengths are filtered and outliers removed. 
%   6DOF osteological ROM is then constrained using ligament thresholds and
%   joint poses exceeding these thresholds removed. The resulting ROM maps 
%   are compared with XROMM data and sensitivity tested using different 
%   thresholds and alpha radii.
%
%   Written by Oliver Demuth 
%
%   Last updated 16.05.2025 - Oliver Demuth

clear;
clc;


%% Load Data

% ligament data (calculated ligament lengths)

RLP2_lig_order = ["LAcH_1", "LAcH_2", "LCoH_dorsalis", "LIcCa", "LScH_dorsalis", "LScH_lateralis", "LScH_posterior"];
RLP3_lig_order = ["LAcH_1", "LAcH_2", "LCoH_dorsalis", "LScH_dorsalis", "LScH_lateralis", "LScH_posterior", "LIcCa"];
RLP4_lig_order = ["LAcH_1", "LAcH_2", "LCoH_dorsalis", "LIcCa", "LScH_dorsalis", "LScH_lateralis", "LScH_posterior"];

[RLP2_lig_order , RLP2_order] = sort(RLP2_lig_order);
[RLP3_lig_order , RLP3_order] = sort(RLP3_lig_order);
[RLP4_lig_order , RLP4_order] = sort(RLP4_lig_order);

RLP2_lig_Data = importDataset('RLP2_new_lig_Trial_000',1,2);
RLP2_lig_Data = RLP2_lig_Data(:,RLP2_order);
RLP3_lig_Data = importDataset('RLP3_new_lig_Trial_000',1,7);
RLP3_lig_Data = RLP3_lig_Data(:,RLP3_order);
RLP4_lig_Data = importDataset('RLP4_new_lig_Trial_000',1,2);
RLP4_lig_Data = RLP4_lig_Data(:,RLP4_order);

RLP2_XROMM = csvread('RLP2_XROMM_rot.csv',0,0);
RLP2_XROMM_trans = csvread('RLP2_XROMM_trans.csv',0,0);
RLP3_XROMM = csvread('RLP3_XROMM_rot.csv',0,0);
RLP3_XROMM_trans = csvread('RLP3_XROMM_trans.csv',0,0);
RLP4_XROMM = csvread('RLP4_XROMM_rot.csv',0,0);
RLP4_XROMM_trans = csvread('RLP4_XROMM_trans.csv',0,0);

trialLength = gcd(length(RLP2_XROMM),length(RLP3_XROMM));

XROMM_trans = vertcat(RLP2_XROMM_trans,RLP3_XROMM_trans,RLP4_XROMM_trans);

% transform euler angles into 2neu coordinate system

Offset = Maya2TransM([90,0,90]); % offset between Maya coordinate system and 2neu coordinate system
[RLP2_mob,RLP2_shp,RLP2_map] =  calcMobility(RLP2_XROMM,'sin',25,false,Offset);
[RLP3_mob,RLP3_shp,RLP3_map] =  calcMobility(RLP3_XROMM,'sin',25,false,Offset);
[RLP4_mob,RLP4_shp,RLP4_map] =  calcMobility(RLP4_XROMM,'sin',25,false,Offset);

RLPs_map = vertcat(RLP2_map,RLP3_map,RLP4_map);
RLPs_shp = alphaShape(RLPs_map,25);

% filter XROMM data

[intRLP2,RLP2f] = filterXROMM(RLP2_map,RLP2_lig_Data,trialLength,'movmedian',20);
[intRLP3,RLP3f] = filterXROMM(RLP3_map,RLP3_lig_Data,trialLength,'movmedian',20);
[intRLP4,RLP4f] = filterXROMM(RLP4_map,RLP4_lig_Data,trialLength,'movmedian',20);

% normalise ligament lengths by minimum humeral circumference

RLP2_HC = 14.06;
RLP3_HC = 14.83;
RLP4_HC = 13.49; 
RLP5_HC = 14.2;

% ligament names

ligaments = ["LAcH", ...
             "LCoHd", ...
             "LIcCa", ...
             "LScHd", ...
             "LScHl", ...
             "LScHp"];

% combine ligament data

ligDataNorm = vertcat(intRLP2/RLP2_HC,intRLP3/RLP3_HC,intRLP4/RLP4_HC);

% average LAcH

ligData = ligDataNorm;
ligData(:,2) = mean(ligDataNorm(:,1:2),2);
ligData(:,1) = [];

% get maximum ligament lengths from XROMM data

maxXROMM = max(ligData);
minXROMM = min(ligData);

% box plot of ligament data

% scaled RLP5 dissection data 

dissectionVals = [14.5, ... % LAcH
                   7.9, ... % LCoHd
                   7.3, ... % LIcCa
                  12.3, ... % LScHd
                   7.2, ... % LScHl
                  12.5];    % LScHp

scaleFactor = 1.26;

fig = figure('Position',[100 100 1200 600]);
boxGroups = vertcat(repmat("RLP2",length(RLP2_lig_Data),1),repmat("RLP3",length(RLP3_lig_Data),1),repmat("RLP4",length(RLP4_lig_Data),1),repmat("combined",length(ligData),1));
boxColours = ['#B0191D';'#40752D';'#576FB6';'#bebebe'];

maxValues = maxXROMM./(dissectionVals/RLP5_HC);
minValues = minXROMM./(dissectionVals/RLP5_HC);
maxPercent = round((maxValues-1)*100,1);

t=tiledlayout(1,6);
t.TileSpacing = 'tight';
t.Padding = 'tight';
for i = 1:size(ligData,2)
    ax = nexttile;
    boxchart(ax,repmat(categorical(ligaments(i)),1,2*length(ligData(:,i))),vertcat(ligData(:,i),ligData(:,i)),'Group',boxGroups,'JitterOutliers','on','MarkerStyle','.','MarkerSize',0.5,'BoxWidth',0.75);
    colororder(boxColours);
    hold on
    yline(dissectionVals(i)/RLP5_HC,'Color','black',LineWidth=1);
    text(2,1.02*dissectionVals(i)/RLP5_HC,append('RLP5',newline,'0%'),'Color','black','HorizontalAlignment','right');
    yline(dissectionVals(i)/RLP5_HC*scaleFactor,'Color',hex2rgb('#7f7f7f'),LineWidth=1);
    text(2,dissectionVals(i)/RLP5_HC*scaleFactor+0.02*dissectionVals(i)/RLP5_HC,append('RLP5',newline,'+ 26%'),'Color',hex2rgb('#7f7f7f'),'HorizontalAlignment','right');
    yline(max(ligData(:,i)),'Color',hex2rgb('#bebebe'),LineWidth=1);
    text(2,max(ligData(:,i))+0.02*dissectionVals(i)/RLP5_HC,append('max. XROMM',newline,num2str(maxPercent(i)),'%'),'Color',hex2rgb('#7f7f7f'),'HorizontalAlignment','right');
    yline(dissectionVals(i)/RLP5_HC*1.17,'Color',hex2rgb('#7f7f7f'),LineWidth=0.5);
    yline(dissectionVals(i)/RLP5_HC*1.47,'Color',hex2rgb('#7f7f7f'),LineWidth=0.5);

    ylim([0 dissectionVals(i)/RLP5_HC*1.5])
    
end

ylabel(t,'Norm. ligament [length/HC]')
lgd = legend('RLP2','RLP3','RLP4','combined','RLP5');
lgd.Layout.Tile = 'south';
lgd.Orientation = 'horizontal';

%exportgraphics(fig,'Plots/RLPs_lig_plot_new.pdf','ContentType','vector');

%% plot normalised ligament lengths across XROMM data

ligColours = ["#418caf",...
              "#24ac73",...
              "#954c98",...
              "#f2be55",...
              "#f18556",...
              "#cc3f64"];

plotInterval = 30;
minRot = floor(min(RLPs_map)/plotInterval)*plotInterval;
maxRot = ceil(max(RLPs_map)/plotInterval)*plotInterval;

for i = 1:size(ligData,2)
    fig = figure();
    
    % create colour map 

    c1 = rgb2hsv(hex2rgb('#d8d8d8'));
    c2 = rgb2hsv(hex2rgb(char(ligColours(i))));
    cmap = (hsv2rgb([repmat(c2(1),1,256); ...
                     pchip([0,0.333,1],[0,0,c2(2)],linspace(0,1,256)); ...
                     spline([0,0.5,1],[0.5,c1(3),c2(3)],linspace(0,1,256))]'));

    % plot data

    scatter3(RLPs_map(:,1),RLPs_map(:,2),RLPs_map(:,3),1,ligData(:,i),'.')
    
    hold on;

    scatter3(RLPs_map(:,1),RLPs_map(:,2),repmat(minRot(3),[length(RLPs_map),1]),1,ligData(:,i),'filled','MarkerEdgeAlpha', 0.15, 'MarkerFaceAlpha', 0.15);

    xl = minRot(1):plotInterval:maxRot(1);
    yl = minRot(2):plotInterval:maxRot(2);
    zl = minRot(3):plotInterval:maxRot(3);
    
    hold on
    
    patch([minRot(1) minRot(1) maxRot(1) maxRot(1)],...
          [minRot(2) maxRot(2) maxRot(2) minRot(2)],...
          [minRot(3) minRot(3) minRot(3) minRot(3)],"white","LineStyle","none","FaceAlpha",1);
    
    plot_euler_lines(xl,yl,minRot(3),[0.85 0.85 0.85],0.5,'sin');
    
    xlim([minRot(1) maxRot(1)])
    ylim([minRot(2) maxRot(2)])
    zlim([minRot(3) maxRot(3)])
    
    hold off
    
    xticks(xl)
    yticks(yl)
    zticks(zl)
    
    xlabel('FE_s_c');
    ylabel('ABAD');
    zlabel('LAR');
    
    view (45,25);
    
    grid on;
    axis on;
    
    colormap(cmap);
    cbar = colorbar;
    cbar.Label.String = append('Norm. ', char(ligaments(i)),' [length/HC]');

    ax = gca;
    ax.FontSize = 15;

    %exportgraphics(fig,append('Plots/RLPs_',char(ligaments(i)),'_XROMM.pdf'),'ContentType','vector');

end


%% import ROM simulation data and average LAcH

RLP2_ROM = csvread('RLP2_ROM_simulation.csv',0,0);
RLP3_ROM = csvread('RLP3_ROM_simulation.csv',0,0);
RLP4_ROM = csvread('RLP4_ROM_simulation.csv',0,0);

RLP2_ROM_lig = importDataset('RLP2_ROM_new_lig_part_',1,17);
RLP2_ROM_lig = RLP2_ROM_lig(:,RLP2_order);
RLP2_ROM_lig(:,2) = mean(RLP2_ROM_lig(:,1:2),2);
RLP2_ROM_lig(:,1) = [];
RLP2_ROM_lig = vertcat(RLP2_ROM_lig(end,:),RLP2_ROM_lig);

RLP3_ROM_lig = importDataset('RLP3_ROM_new_lig_part_',1,17);
RLP3_ROM_lig = RLP3_ROM_lig(:,RLP3_order);
RLP3_ROM_lig(:,2) = mean(RLP3_ROM_lig(:,1:2),2);
RLP3_ROM_lig(:,1) = [];
RLP3_ROM_lig = vertcat(RLP3_ROM_lig(end,:),RLP3_ROM_lig);

RLP4_ROM_lig = importDataset('RLP4_ROM_new_lig_part_',1,17);
RLP4_ROM_lig = RLP4_ROM_lig(:,RLP4_order);
RLP4_ROM_lig(:,2) = mean(RLP4_ROM_lig(:,1:2),2);
RLP4_ROM_lig(:,1) = [];
RLP4_ROM_lig = vertcat(RLP4_ROM_lig(end,:),RLP4_ROM_lig);

% average LAcH

alpha = sqrt(3)*5;

[RLP2_ROM_mob,~,RLP2_ROM_map] = calcMobility(RLP2_ROM,'sin',alpha,false,Offset);
[RLP3_ROM_mob,~,RLP3_ROM_map] = calcMobility(RLP3_ROM,'sin',alpha,false,Offset);
[RLP4_ROM_mob,~,RLP4_ROM_map] = calcMobility(RLP4_ROM,'sin',alpha,false,Offset);

[RLPs_mobility,~] = get_mobility(RLPs_map,25,false); % from Lee et al. 2023

%% calculate missing poses

[intRLP2ROM,RLP2ROMf] = interpOutliers(RLP2_ROM_map,RLP2_ROM_lig);
[intRLP3ROM,RLP3ROMf] = interpOutliers(RLP3_ROM_map,RLP3_ROM_lig);
[intRLP4ROM,RLP4ROMf] = interpOutliers(RLP4_ROM_map,RLP4_ROM_lig);

%% calculate maximum ligament lengths of XROMM poses in ROM map

[maxRLP2Lig,maxRLP3Lig,maxRLP4Lig] = deal(zeros(1,length(RLP2ROMf)));
for i = 1:length(RLP2ROMf)
    maxRLP2Lig(i) = max(RLP2ROMf{i}(RLPs_map));
    maxRLP3Lig(i) = max(RLP3ROMf{i}(RLPs_map));
    maxRLP4Lig(i) = max(RLP4ROMf{i}(RLPs_map));
end


%%

ligThresholds = zeros(4,6,3);

% maximum interpolated values based on XROMM data

ligThresholds(1,:,1) = maxRLP2Lig;
ligThresholds(1,:,2) = maxRLP3Lig;
ligThresholds(1,:,3) = maxRLP4Lig;

% scaled dissection values + 26 % strain (mean strain of Baier et al. 2012)

ligThresholds(2,:,1) = dissectionVals / RLP5_HC * RLP2_HC * scaleFactor;  
ligThresholds(2,:,2) = dissectionVals / RLP5_HC * RLP3_HC * scaleFactor; 
ligThresholds(2,:,3) = dissectionVals / RLP5_HC * RLP4_HC * scaleFactor; 

% scaled XROMM values

ligThresholds(3,:,1) = maxXROMM * RLP2_HC;  
ligThresholds(3,:,2) = maxXROMM * RLP3_HC;  
ligThresholds(3,:,3) = maxXROMM * RLP4_HC;  

% maxValues

ligThresholds(4,:,:) = repmat(100,6,3);  

% calculate and plot missed volumes

[vols,res] = deal(zeros(4,3));

figs = cell(3,4);
shapes = cell(3,4);
centroids = zeros(3,4,3);
meassurements = zeros(3,4,6);

specimens = ["RLP2","RLP3","RLP4"];
dataStr = ["max ROM", "dissection", "XROMM", "max values"];

ROMmaps = {RLP2_ROM_map,RLP3_ROM_map,RLP4_ROM_map};
intData = {intRLP2ROM,intRLP3ROM,intRLP4ROM};

for i = 1:length(specimens)
    for j = 1:length(dataStr)
        [figs{i,j}, res(j,i), vols(j,i), shapes{i,j}, ~] = plotMissed(intData{i},ROMmaps{i},RLPs_map,ligThresholds(j,:,i),specimens(i)+" "+dataStr(j),false,sqrt(3)*5);
        centroids(i,j,:) = calc3DCentroid(shapes{i,j});
        for k = 1:length(RLP2ROMf)
            if i == 1
                meassurements(i,j,k) = RLP2ROMf{k}(centroids(i,j,:))/RLP2_HC;
            elseif i == 2
                meassurements(i,j,k) = RLP3ROMf{k}(centroids(i,j,:))/RLP3_HC;
            else 
                meassurements(i,j,k) = RLP4ROMf{k}(centroids(i,j,:))/RLP4_HC;
            end
        end
    end
end

resPerc = 100/RLPs_mobility*res;


%% sensitivity analysis of alpha shape radii

intervals = 0.5:0.5:25;
volumina = zeros(length(specimens),length(dataStr),length(intervals));

for i = 1:length(specimens)
    for j = 1:length(dataStr)
        for k = 1:length(intervals)
            shp = shapes{i,j};
            shp.Alpha = intervals(k);
            volumina(i,j,k) = volume(shp);
        end
    end
end

XROMMshapes = {RLP2_shp,RLP3_shp,RLP4_shp,RLPs_shp};

XROMMintervals = 0.5:0.5:100;
XROMMvolumina = zeros(4,length(XROMMintervals));
for i = 1:length(XROMMshapes)
    for j = 1:length(XROMMintervals)
        shp = XROMMshapes{i};
        shp.Alpha = XROMMintervals(j);
        XROMMvolumina(i,j) = volume(shp);
    end
end

f = figure();
figureSize = f.Position;
figureSizeNew = figureSize;
figureSizeNew(4) = figureSizeNew(4)*1.5;
f.Position = figureSizeNew;
t = tiledlayout(3,2);
t.Padding = 'compact';
t.TileSpacing = 'compact';

lineColours = ['#B0191D';'#40752D';'#576FB6';'#bebebe'];
for j = 1:5
    if j == 1
        nexttile([1 2])
        for i = 1:4
            plot(XROMMintervals,squeeze(XROMMvolumina(i,:)),'Color',lineColours(i,:),'LineWidth',1.5)
            hold on;
        end
        xline(25)
        hold off;
        title('ex vivo');
        xlabel('Alpha radius');
        ylabel('Volume [°^3]');
        ylim([0,7e+05])
    else
        nexttile
        for i = 1:length(specimens)
            plot(intervals,squeeze(volumina(i,j-1,:)),'Color',lineColours(i,:),'LineWidth',1.5)
            hold on;
        end
        xline(sqrt(3)*5)
        hold off;
        title(dataStr(j-1));
        xlabel('Alpha radius');
        ylabel('Volume [°^3]');
    end
end

lgd = legend('RLP2','RLP3','RLP4','combined');
lgd.Location = 'southeast';

%%
for i = 1:size(figs,2)
    f1=figure('units','normalized','outerposition',[0 0 1 0.75]);
    t=tiledlayout(1,3,'Parent',f1);  %Create tiled layout
    t.TileSpacing = 'tight';
    t.Padding = 'tight';

    tile = 1:3;
    for j = 1:size(figs,1)
        axis equal
        lgd=findobj(figs{j,i},'Type','Legend');
        set(lgd, 'visible', 'off');
        ax = findobj(figs{j,i},'Type','axes');
        if i == 1
            title(ax, specimens(j));
        end
        ax.Parent=t;
        ax.Layout.Tile = tile(j);
    end
    
    set(lgd, 'visible','on');
    lgd.Layout.Tile = 'south';
    lgd.Orientation = 'horizontal';
    
    set(gca,'visible','off')
    test = 1;
    %exportgraphics(f1,append('Plots/RLPs_ROM_plot_',num2str(i),'.pdf'),'ContentType','vector');
end



%% ==== Custom Helper Functions ==== %%


%% Helper functions to streamline calulcations

% calculate mobility parameters from Maya joint angles

function [mobility,shp,map] = calcMobility(Data,type,rad,plot,Offset)
    if nargin < 5
        transMat = Maya2TransM(Data);
    else
        transMat = Maya2TransM(Data,Offset);
    end
    
    [plane, abd, axial] = get_spherical_angles_2neu(transMat); % from Lee et al. 2023
    map = euler2map([plane, abd, axial],type); % from Lee et al. 2023
    [mobility,shp] = get_mobility(map,rad,plot); % from Lee et al. 2023
end

% get Maya rotation from 2neu

function rot = map2Maya(map,OffsetMat)
    
    if nargin < 2 
        OffsetMat = eye(4);
    end

    map_2neu = [map(:,1)/sind(map(:,2)),map(:,2),map(:,3)];
    rotMat = get_R_spherical_2neu(map_2neu(:,1),map_2neu(:,2),map_2neu(:,3)); % from Lee et al. 2023
    corrMat = rotMat/OffsetMat;
    rot = flip(rad2deg(rotm2eul(corrMat(1:3,1:3))));
end

% filter XROMM data function

function [intDataF,intF] = filterXROMM(map,data,trialLength,method,window)

% extract ligament data and interpolate outliers

[intData,intF] = interpOutliers(map,data,method,window);

% filter/smooth XROMM data using robust lowess

dataDims = size(data);
intDataInd = reshape(intData,trialLength,dataDims(2),dataDims(1)/trialLength);
intDataF = intDataInd;

for i = 1:size(intDataInd,2)
    %[b,a] = butter(4,cutOff/(frameRate/2));
    intDataF(2:end,i,:) = smoothdata(intDataInd(2:end,i,:),'rlowess',window/2); % skip first row as this is the CT position
    %intDataF(2:end,i,:) = filtfilt(b,a,filtData); % filter using buttworth low pass filter
end

intDataF = reshape(intDataF,size(intData));

end


% interpolate outliers function

function [intData,fCell] = interpOutliers(indData,depData,method,window)

    if nargin == 4
        outliers = (depData(:,:) < 0 | isoutlier(depData(:,:), method, window, 1)); 
    elseif nargin == 2
        outliers = depData(:,:) < 0;
    else
        error('Please supply two (indData, depData) or four (indData, depData, method, window) input arguments');
    end

    intData = zeros(size(depData));
    fCell = cell(1,size(depData,2));

    for i = 1:size(depData,2)

        % remove outliers

        indpOut = indData(~outliers(:,i),:);
        depOut = depData(~outliers(:,i),i);
        
        % interpolate missing values

        f = scatteredInterpolant(indpOut(:,1),indpOut(:,2),indpOut(:,3),depOut,'natural');
        fCell{i} = f;
        intData(:,i) = f(indData);

        % check if interpolated values are below Euclidean distance (i.e.,
        % impossible)
        
        belowEuc = (depData(:,i) < 0) & (intData(:,i) < abs(depData(:,i)));
        intData(belowEuc,i) = abs(depData(belowEuc,i));

    end
end

% plot missing poses function

function [missedVolFig, missedVolume, totalVolume, expShp, expMap] = plotMissed(intData,indData,refData,threshold,dataStr,denoise,alpha)

% drop data points above threshold

dropInt = ~any((intData > threshold | intData < 0),2); % remove data points that are beyond max values
expMap = indData(dropInt,:);

% filter point cloud and remove 'islands'

ptCloud = pointCloud(expMap);
if denoise
    ptCloud = pcdenoise(ptCloud);
end

[labels,numClusters] = pcsegdist(ptCloud,alpha); % define clusters, min distance is diagonal of a cube with edge length of 5 (i.e., 5 degree interval in Maya)

ClusterBins = (1:numClusters); %number of clusters
ClusterElementCount = histcounts(labels,numClusters); %elements within each cluster
ElementsPerBin = transpose([ClusterBins; ClusterElementCount]);
ClustersFilteredNum = ElementsPerBin(find(ElementsPerBin(:,2) >= max(ClusterElementCount),1,'first'),:); % only retains points within largerst cluster
Cluster = select(ptCloud,find(labels == ClustersFilteredNum(:,1))); % select largest cluster

expMap = Cluster.Location;
expShp = alphaShape(expMap,alpha);
totalVolume = volume(expShp);

% get missed data points

idOut=~inShape(expShp,refData(:,1),refData(:,2),refData(:,3));
missedPoints = [refData(idOut,1), refData(idOut,2), refData(idOut,3)];
missedShape = alphaShape(missedPoints,25);
missedVolume = volume(missedShape);

refShp = alphaShape(refData,25);

% plot figure

missedVolFig = figure;
hold on

set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

Shape1 = plot(missedShape,'DisplayName','Missed data'); %plot the first alpha shape on the same axes 
set(Shape1, 'facecolor','#B0191D','edgecolor','#B0191D',  'facealpha', '0.95', 'edgealpha', '0.95'); %set colour and opacity of alpha shape
Shape2 = plot(expShp,'DisplayName','constrained ROM data'); %plot the second alpha shape on the same axes 
set(Shape2, 'facecolor','#40752D','edgecolor','#40752D',  'facealpha', '0.05', 'edgealpha', '0.05'); %set colour and opacity of alpha shape
Shape3 = plot(refShp,'DisplayName','XROMM data'); %plot the second alpha shape on the same axes 
set(Shape3, 'facecolor','#576FB6','edgecolor','#576FB6',  'facealpha', '0.5', 'edgealpha', '0.5'); %set colour and opacity of alpha shape

minLim = min(expMap);
maxLim = max(expMap);

interval = 60;

xMin = floor(minLim(1)/interval)*interval;
xMax = ceil(maxLim(1)/interval)*interval;
yMin = floor(minLim(2)/interval)*interval;
yMax = ceil(maxLim(2)/interval)*interval;
zMin = floor(minLim(3)/interval)*interval;
zMax = ceil(maxLim(3)/interval)*interval;

currentHandles = [Shape1, Shape2, Shape3]; % Add handle to the list

xl = xMin:interval:xMax;
yl = yMin:interval:yMax;

hold on

patch([xMin xMin xMax xMax],...
      [yMin yMax yMax yMin],...
      [zMin zMin zMin zMin],"white","LineStyle","none","FaceAlpha",1);


plot_euler_lines(xl,yl,zMin,[0.85 0.85 0.85],0.5,'sin');

xlim([xMin xMax])
ylim([yMin yMax])
zlim([zMin zMax])

hold off

xticks(xMin:interval:xMax)
yticks(yMin:interval:yMax)
zticks(zMin:interval:zMax)

xlabel('FE_s_c');
ylabel('ABAD');
zlabel('LAR');

legend(currentHandles);

view (45,25);

grid on;
axis on;

temp_ax = gca;
temp_ax.FontSize = 15;


end

% calculate centroid of alpha shape

function Centroid = calc3DCentroid(Datashape)

[tri,pts] = boundaryFacets(Datashape);
TR = triangulation(tri, pts);

numTriangles3D = size(TR,1);

Areas3D = zeros(numTriangles3D,1);
Centres3D = zeros(numTriangles3D,3);
    for i=1:numTriangles3D
        vertex1=[pts(TR.ConnectivityList(i,1),1),pts(TR.ConnectivityList(i,1),2),pts(TR.ConnectivityList(i,1),3)];
        vertex2=[pts(TR.ConnectivityList(i,2),1),pts(TR.ConnectivityList(i,2),2),pts(TR.ConnectivityList(i,2),3)];
        vertex3=[pts(TR.ConnectivityList(i,3),1),pts(TR.ConnectivityList(i,3),2),pts(TR.ConnectivityList(i,3),3)];

        %Compute area of each triangle using cross product
        Areas3D(i) = 0.5*norm(cross(vertex2-vertex1,vertex3-vertex1));

        %Compute centroid of each triangle
        Centres3D(i,:) = (vertex1(1:3)+vertex2(1:3)+vertex3(1:3))/3;
    end

TotalArea = sum(Areas3D);
Centroid = [sum((Centres3D(:,1).*Areas3D))/TotalArea, sum((Centres3D(:,2).*Areas3D))/TotalArea, sum((Centres3D(:,3).*Areas3D))/TotalArea];  

end

% converts hexadecimal color code into RGB triplet function

function rgb = hex2rgb(hexArray)

    rgb = zeros(size(hexArray,1),3);
    for i=1:size(hexArray,1)
        temp_col = hexArray(i,:);
        rgb(i,:) = sscanf(temp_col(2:end),'%2x%2x%2x',[1 3])/255;
    end
end

% import ligament data

function Data = importDataset(prefix,first,last)
    numRange = first:1:last;
    
    % go through specified ligament files
    for i = 1:length(numRange)
        
        % import data from ligament part csv files
    
        filename = string(prefix) + int2str(numRange(i)) + ".csv";
        
        if i == 1
            Data = csvread(filename,1,0);
        else
            Data2 = csvread(filename,1,0);
            Data = vertcat(Data,Data2);
        end
    end
end
