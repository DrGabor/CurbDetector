function HDLInfo = distanceFilter(HDLInfo0, AngN)
HDLInfo = HDLInfo0; 
AngRange = [];
for i = 1 : 1 : AngN
    AngRange(end+1, :) = [360.0/AngN*(i-1) 360.0/AngN*i];
end
for id = 1 : 1 : length(HDLInfo)
    if id == 23
        bTest = 1; 
    end
    LayerInfo = HDLInfo(id);
    data = LayerInfo.data;
    curbPts = LayerInfo.curbPts;
    if isempty(curbPts)
        continue;
    end
    Idx = []; 
    for i = 1 : 1 : size(curbPts, 2)
        [NNIdx, DD] = knnsearch(curbPts(1:3, i)', data(1:3, :)');
        tmpIdx = find(DD == 0); 
        Idx(end+1) = tmpIdx(1);
    end
    Ang = LayerInfo.Ang(Idx);
    rstPts = [];
    for i = 1 : 1 : size(AngRange, 1)
        tmpRange = AngRange(i, :);
        Idx = find(Ang >= tmpRange(1) & Ang < tmpRange(2));
        if isempty(Idx)
            continue;
        end
        pts = curbPts(:, Idx);
        [~, minIdx] = min(abs(pts(2, :)));
        rstPts(:, end+1) = pts(:, minIdx);
    end
    HDLInfo(id).curbPts = rstPts;
end