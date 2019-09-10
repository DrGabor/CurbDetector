clc; close all; clear all;
addpath('D:\MatlabPros\CommonFunctions\Velodyne');
DataFolder = 'D:\MatlabPros\Curb4Journal\CurbDetector_GitHub\data'; 
List = dir( fullfile(DataFolder, '*.txt') ); 
TMethod = {'Zhang', 'Hata', 'Chen'};
%%%%%% the vertical angle of HDL-64E. 
load VertAng.mat
%%%%%% the installation height of HDL-64E. 
H = 1.75;
CurbInfo = [];
hh = figure;
for nFrm = 1 : 1 : length(List)
    %%%%%%%% load data.
    DataDir = fullfile( DataFolder, List(nFrm).name );
    pcData = HDLS3AnalyserFun(DataDir);
    Dist = sqrt(pcData(1, :).^2 + pcData(2, :).^2);
    EffIdx = find( abs( pcData(1, :) ) <= 40.0 & abs(pcData(2, :)) <= 20.0 & Dist >= 2.5 );
    pcData = pcData(:, EffIdx);
    %%%%%%%%% make sure the last second row should be the layer index.
    LayerArray = pcData(end-1, :);
    % Organize point cloud as polar coordinate.
    HDLInfo0 = ReArrangeHDLFun( pcData, 0 );
    %% curb detector. 
    for selId = 1 : 1 : length(TMethod)
        Method = TMethod{selId};
        %% calculate feature.
        time_a = tic;
        if strcmp(Method, 'Zhang')
            N_Neigh = 15;
            hAngRes = 0.09*4.0;  % is the honrizonal angle resolution of each ring.
            t = 0.3;
            [B, P, inliers] = ransacfitplane(pcData(1:3, :), t, 0);
            HDLInfo_Plane = ReArrangeHDLFun( pcData(:, inliers), 0 );
            HDLInfo = Zhang(HDLInfo_Plane, N_Neigh, H, hAngRes, VertAng);
        end
        if strcmp(Method, 'Hata')
            AngRes = 3.0;
            hDiffThr = 0.15;
            HDLInfo = Hata(HDLInfo0, AngRes, hDiffThr);
        end
        if strcmp(Method, 'Chen')
            smoothThr = 0.2;
            N_Neigh = 5;
            CurbThr = [0.05 0.5];
            HDLInfo = Chen(HDLInfo0, N_Neigh, smoothThr,CurbThr);
        end
        %% distance filter.
        AngN = 4;
        HDLInfo = distanceFilter(HDLInfo, AngN);
        CurbPts = cat(2, HDLInfo(:).curbPts);
        nTime = toc(time_a);
        %% accumulate curb points.
        tmp = [];
        tmp.nFrm = nFrm;
        tmp.method = Method;
        tmp.curb = CurbPts;
        tmp.time = nTime;
        CurbInfo = [CurbInfo tmp];
        %% calculate precison and recall.
        str = sprintf('nFrm = %04d, method = %s, \ncurbPtsNum = %03d', ...
            nFrm, Method, size(CurbPts, 2) );
        %% plot.
        figure(hh);
        cla;
        hold on;
        grid on;
        axis equal;
        box on;
        pcshow(pcData(1:3, :)');
        CurbPts = cat(2, HDLInfo(:).curbPts);
        if ~isempty(CurbPts)
            plot3(CurbPts(1, :), CurbPts(2, :), CurbPts(3, :), ...
                'linestyle', 'none', 'marker', 'd', 'markersize', 4, ...
                'markerfacecolor', 'r', 'markeredgecolor', 'r');
        end
        tmpPts = [];
        tmpPts(:, end+1) = [-50; -30; -2.5];
        tmpPts(:, end+1) = [50; 30; 0.0];
        pcshow(tmpPts', 'k');
        axis([-50 50 -30 30]);
        set(gca, 'Clipping', 'on');
        view([0 90]);
        xlabel('X/m');
        ylabel('Y/m');
        title(str);
        disp(sprintf('%s, time = %04dms', str, round(nTime*1000.0)));
        bTest = 1;
    end
end