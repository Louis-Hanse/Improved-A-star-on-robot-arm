function output = genRandomPose()
    mdl_puma560
    minLim = p560.qlim(:,1);
    rangeLim = p560.qlim(:,2) - p560.qlim(:,1);
    output = zeros(1,6);
    for i = 1:6;
        output(i) = rand()*rangeLim(i) - minLim(i);
    end
end