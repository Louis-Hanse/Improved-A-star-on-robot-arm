function path = buildPath(node,nodeList)
    path = {};
    curNode = node;
    pathCache = {curNode};
    
    while ~max(isnan(curNode.parent))
        curNode = nodeList{findCord(nodeList,curNode)};
        pathCache{end+1} = curNode;
    end
    for i = size(pathCache,2):-1:1
        path{end+1} = pathCache{i};
    end
end

%%  从列表中寻找指定坐标的节点
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