function output = isOutOfLim(pose)
    qLim = [
        -2.7925 2.7925;
        -0.7854 3.9270;
        -3.9270 0.7854;
        -1.9199 2.9671;
        -1.7453 1.7453;
        -4.6426 4.6426;
    ];
    output = false;
    for i = 1:6
        if ~inLim(pose(i),qLim(i,:))
            output = true;
            break;
        end
    end
end

function output = inLim(angle,rangeList)
    output = false;
    if angle < min(rangeList)
        while angle < min(rangeList)
            angle = angle + 2*pi;
        end
    elseif angle > max(rangeList)
        while angle > max(rangeList)
            angle = angle - 2*pi;
        end
    else
        output = true;
    end

    if ~output
        if angle < max(rangeList) && angle > min(rangeList)
            output = true;
        end
    end
end