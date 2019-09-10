function [ HDLInfo ] = ReArrangeHDLFun( pcData, USE_DESAMPLE )
HDLInfo = [];
for nLayerId = 1 : 1 : 64
    %%%%%% Re-arrange data.
    LayerInfo = [];
    EffIdx = find(pcData(end-1, :) == nLayerId);
    data = pcData(:, EffIdx);
    Ang = atan2d(data(2, :), data(1, :));
    Ang = wrapTo360(Ang);
    [Ang, Idx_sort] = sort(Ang, 'ascend');
    data = data(:, Idx_sort); 
    if USE_DESAMPLE    
        A = round(Ang/0.25);
        C = unique(A);
        AngNew = [];
        dataNew = [];
        for i = 1 : 1 : length(C)
            if i == 1
                bTest = 1;
            end
            Idx = find(A == C(i));
            Dist = sum(data(1:3, Idx).^2 );
            [~, id0] = min(Dist);
            Id = Idx(id0);
            AngNew(end+1) = Ang(Id);
            dataNew(:, end+1) = data(:, Id);
            bTest = 1;
        end
        Ang = AngNew;
        data = dataNew;
    end
    LayerInfo.id = nLayerId;
    LayerInfo.data = data;
    LayerInfo.Ang = Ang;
    LayerInfo.idx = EffIdx(Idx_sort); 
    HDLInfo = [HDLInfo LayerInfo];
end
end

