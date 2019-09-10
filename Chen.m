function HDLInfo = Chen(HDLInfo0, N_Neigh, smoothThr,CurbThr)
HDLInfo = HDLInfo0; 
for nLayerId = 1 : 1 : length(HDLInfo)
    LayerInfo = HDLInfo(nLayerId);
    data = LayerInfo.data;
    Ang = LayerInfo.Ang;
    N = size(data, 2);
    curvature = [];
    hDiff = [];
    for i = 1 : 1 : N
        idx0 = mod( i-(N_Neigh:-1:1), N);
        idx0(idx0==0) = N;
        idx1 = mod( i+(1:1:N_Neigh), N);
        idx1(idx1==0) = N;
        Idx = [idx0 i idx1];
        tmp = data(1:2, Idx);
        tmp = bsxfun(@minus, data(1:3, Idx), data(1:3, i));
        %%%%%%% calculate difference vector.
        tmp = bsxfun(@minus, data(1:3, Idx), data(1:3, i));
        tmp = sum(tmp, 2);
        curvature(end+1) = norm(tmp)^2; %norm(tmp)/norm(data(1:3, i))/length(Idx);    % norm(tmp/10);  sum(tmp.^2);% sum(tmp.^2);
        S = [data(1:3, Idx)];
        hDiff(end+1) = max(S(3, :)) - min(S(3, :));
        bTest = 1;
    end
    smoothArc = [];
    arcLenThr = max(N_Neigh - 2, 3);
    for i = 1 : 1 : N
        idx0 = mod( i-(N_Neigh:-1:1), N);
        idx0(idx0==0) = N;
        idx1 = mod( i+(1:1:N_Neigh), N);
        idx1(idx1==0) = N;
        flag = 0;
        if sum(curvature(idx0) <= smoothThr) > arcLenThr | ...
                sum(curvature(idx1) <= smoothThr) > arcLenThr
            flag = 1;
        end
        smoothArc(end+1) = flag;
    end
    HDLInfo(nLayerId).curvature = curvature;
    HDLInfo(nLayerId).hDiff = hDiff;
    HDLInfo(nLayerId).smoothArc = smoothArc;
    bTest = 1;
end
for id = 1 : 1 : length(HDLInfo)
    data = HDLInfo(id).data;
    curvature = HDLInfo(id).curvature;
    hDiff = HDLInfo(id).hDiff;
    smoothArc = HDLInfo(id).smoothArc;
    EffIdx = find( curvature >= smoothThr & smoothArc & hDiff >= CurbThr(1) & hDiff <= CurbThr(2) );
    curbPts = data(1:3, EffIdx);
    HDLInfo(id).curbPts = curbPts;
end