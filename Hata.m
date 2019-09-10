function HDLInfo = Hata(HDLInfo0, AngRes, hDiffThr )
HDLInfo = HDLInfo0; 
N = round(360.0/AngRes);
TRing = [];
for nLayerId = 1 : 1 : length(HDLInfo)
    LayerInfo = HDLInfo(nLayerId);
    data = LayerInfo.data;
    Ang = LayerInfo.Ang;
    A = round(Ang/AngRes);
    Ring = zeros(1, N);
    Pts = zeros(3, N);
    for i = 1 : 1 : N
        Idx = find(A == i - 1);
        if  isempty(Idx)
            continue;
        end
        tmp = data(1:3, Idx);
        midPt = mean(tmp(:, :), 2);
        Ring(i) = norm(midPt(1:2));
        [tmpIdx, ~] = knnsearch( tmp', midPt');
        Pts(:, i) = tmp(:, tmpIdx); % mean(tmp, 2);
        bTest = 1;
    end
    tmp = [];
    tmp.pts = Pts;
    tmp.ring = Ring;
    tmp.id = nLayerId;
    TRing = [TRing tmp];
    bTest = [];
end
Alpha = 0.113; 0.001; 
Beta = 0.9; % 1.375;
H = 1.75;
load VertAng.mat
I = cat(1, TRing(:).ring );
%%%%%%% ring component analysis
Flag_RCA = zeros( size(I) );
for id = 1 : 1 : length(TRing)-1
    Ring0 = TRing(id);
    Ring1 = TRing(id+1);
    Pts = TRing(id).pts;
    Diff = abs(Ring0.ring - Ring1.ring);
    DeltaR = H * (cotd(VertAng(id+1)) - cotd(VertAng(id)) );
    DeltaR = abs(DeltaR);
    Idx = find( Diff >= Alpha * DeltaR & Diff < Beta * DeltaR ...
        & Ring0.ring ~= 0 & Ring1.ring ~= 0 );
    Flag_RCA(id, Idx) = 1;
end
%%%%%% Differential filter.
Flag_Diff = zeros(size(I));
for nLayerId = 1 : 1 : size(I, 1)
    N = size(I, 2);
    for i = 1 : 1 : N
        idx = [i-1 i+1];
        idx(idx<=0) = N;
        idx(idx>N ) = 1;
        pt0 = TRing(nLayerId).pts(:, idx(1));
        pt1 = TRing(nLayerId).pts(:, idx(2));
        Diff = 2*( pt0(3) - pt1(3) );
        Diff = abs(Diff);
        if Diff >= hDiffThr
            Flag_Diff(nLayerId, i) = 1;
        end
        bTest = 1;
    end
end
%%%%%% extract curb points for each ring.
for id = 1 : 1 : length(TRing)
    RingInfo = TRing(id);
    flag_rca = Flag_RCA(id, :);
    flag_diff = Flag_Diff(id, :);
    flag = flag_rca & flag_diff;
    curbPts = RingInfo.pts(:, flag);
    HDLInfo(id).curbPts = curbPts;
    bTest = 1;
end