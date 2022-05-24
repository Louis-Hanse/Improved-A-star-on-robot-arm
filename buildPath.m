function pathList = buildPath(start,closeList,robotArm)
    pathList = {};
    node = start;
    while true
        if max(isnan(node.parent))
            break;
        end
        pathList{end+1} = node;
        node = closeList{findCord(closeList,node.parent)};
    end
    disp('Path established!');
    toc;pause;
    pathSize = max(size(pathList));
    for i = pathSize:-1:1
        robotArm.plot(pathList{i}.pose);
        x = pathList{i}.cord(1);
        y = pathList{i}.cord(2);
        z = pathList{i}.cord(3);
        plot3(x,y,z,'ro');
    end
end

function output = findCord(list,cord)
    output = NaN;
    if min(size(list)) == 0
        return;
    end
    for i = 1:max(size(list))
        if list{i}.cord == cord
            output = i;
            break;
        end
    end
end