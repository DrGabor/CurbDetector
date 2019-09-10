function HDLInfo = Zhang(HDLInfo0, N_Neigh, H, hAngRes, VertAng)
HDLInfo = HDLInfo0;
for nLayerId = 1 : 1 : length(HDLInfo)
    LayerInfo = HDLInfo(nLayerId);
    xyThr = abs(H*cotd(VertAng(nLayerId))*deg2rad(hAngRes));
    zThr  = abs(xyThr*tand(VertAng(nLayerId)));
    data = LayerInfo.data;
    N = size(data, 2);
    XY = [];
    Z = [];
    Sita = [];
    hDiff = [];
    Ascend = [];
    for i = 1 : 1 : N
        id0 = i;
        id1 = i+1;
        if id0 == N
            id1 = 1;
        end
        XY(end+1) = norm(data(1:2, id0) - data(1:2, id1));
        Z(end+1)  = abs(data(3, id0) - data(3, id1));
        idx0 = mod( i-(N_Neigh:-1:1), N);
        idx0(idx0==0) = N;
        idx1 = mod( i+(1:1:N_Neigh), N);
        idx1(idx1==0) = N;
        tmpL = bsxfun(@minus, data(1:3, idx0), data(1:3, i));
        tmpR = bsxfun(@minus, data(1:3, idx1), data(1:3, i));
        va = mean(tmpL, 2);
        vb = mean(tmpR, 2);
        va = va/norm(va);
        vb = vb/norm(vb);
        Sita(end+1) = acosd( va'*vb );
        hDiff(end+1) = max( [tmpL(3, :) tmpR(3, :)] );
        NumThr = round(N_Neigh*0.7);
        Ascend(end+1) = sum(tmpL(3, :) >= 0.0) > NumThr | sum(tmpR(3, :) >= 0.0) > NumThr; % issorted(tmpL(3, :), 'ascend') | issorted(tmpR(3, :), 'ascend');
        bTest = 1;
    end
    EffIdx = find( Sita >= 100.0 & Sita <= 160.0 & XY >= xyThr & Z >= zThr ...
        & hDiff >= 0.10 & Ascend ); % find(XY >= xyThr & Z >= zThr & Sita < 160.0) %  & Sita < 140.0 & hDiff >= -1.0 & Ascend);
    curbPts = data(1:3, EffIdx);
    HDLInfo(nLayerId).curbPts = curbPts;
    %         figure;
    %         hold on;
    %         grid on;
    %         plot(Z, 'b.');
    %         figure;
    %         hold on;
    %         grid on;
    %         dcm_obj = datacursormode(gcf);
    %         datacursormode on
    %         set(dcm_obj,'updatefcn',{@pose_callback,Sita, XY, Z, hDiff});
    %         cloud = pointCloud([data(1:3, :)]');
    %         pcshow(cloud);
    %         xlabel('X');
    %         ylabel('Y');
    %         if ~isempty(curbPts)
    %         plot3(curbPts(1, :), curbPts(2, :), curbPts(3, :), ...
    %             'ro', 'markersize', 3, 'markerfacecolor', 'r');
    %         end
    %         str = sprintf('curbPtsNum = %02d', size(curbPts, 2));
    %         title(str);
    % EffIdx = find(XY > xyThr & Z > zThr);% Sita < 150.0 & hDiff >= 0.05
    bTest = 1;
end
end