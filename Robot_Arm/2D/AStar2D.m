clc;clear;
figure(2);clf;
axis equal;
axis off
hold on;
box off

map.grid = [
    'soooooxxxxoooooooooo';
    'ooooooxooxoxoooooooo';
    'ooooooxooxooooxoxooo';
    'ooooooxxoxxoxooxoxoo';
    'xoxoooxoxoooooxoxooo';
    'oxoxoooxoxoxoooxoxoo';
    'xoxoooxoxooooooooxxo';
    'oxoxoooooxoooooooxxo';
    'ooooooooooxoxooooxxo';
    'oooooooooooxoxoooxxo';
    'ooooxoxoooxoxooooxxo';
    'oooooxoxoooxoxoooxxo';
    'ooooxoxooooooooooxxo';
    'oooooxoxoooooooooxxo';
    'oooooooooooooooooxxo';
    'oooooooooooxoxoooxxo';
    'ooooooooooxoxooooxoo';
    'oxoxoooooooxoxoooxoo';
    'ooxoxoooooxoxooooxoo';
    'oxoxoooooooooooooxog';
];

map.size = [size(map.grid,1),size(map.grid,2)];

for i = 1:map.size(1)
    for j = 1:map.size(2)
        switch map.grid(i,j)
        case 's'
            map.start = [i-1,j-1];
        case 'g'
            map.goal = [i-1,j-1];
        case 'x'
            plotBlock([i-1,j-1],'k');
        end
    end
end

[sx,sy] = turnCord(map.start);
startNode.cord = map.start;
startNode.parent = [NaN,NaN];
startNode.cost = 0;
startNode.g = 0;
openList = {startNode};
closeList = {};

while true
    %disp('aaa')
    %disp(openList)
    if size(openList,2) > 0
        minCost = 65536;
        for i = 1:size(openList,2)
            if minCost > openList{i}.cost
                minCost = openList{i}.cost;
                curFlag = i;
                curNode = openList{curFlag};
            end
        end
        if min(curNode.cord == map.goal)
            disp('Goal!')
            break;
        end
        openList(curFlag) = [];
        closeList{end+1} = curNode;
        expList = [1,0; -1,0; 0,1; 0,-1; 1,1; 1,-1; -1,1; -1,-1];
        for i = 1:8
            node.cord = curNode.cord + expList(i,:);
            [x,y] = turnCord(node.cord);
            %disp([x,y])
            if x < 0 || x >= map.size(1) || y < 0 || y >= map.size(2)
                continue
            else
                clFlag = false;
                for j = 1:size(closeList,2)
                    if min(closeList{j}.cord == node.cord)
                        clFlag = true;
                        break;
                    end
                end
                if strcmp(map.grid(x+1,y+1),'x') || clFlag
                    continue
                else
                    colFlag = false;
                    if sum(abs(expList(i,:))) > 1
                        sub_x = curNode.cord + [expList(i,1),0];
                        sub_y = curNode.cord + [0,expList(i,2)];
                        subCord  = [sub_x;sub_y];
                        for j = 1:2
                            [zx,zy] = turnCord(subCord(j,:));
                            if strcmp(map.grid(zx+1,zy+1),'x') || clFlag
                                colFlag = true;
                            end
                        end
                    end
                    if ~colFlag
                        node.parent = curNode.cord;
                        node.g = curNode.g + round(norm(expList(i,:)),1);
                        node.cost =  node.g + sum(abs(node.cord - map.goal));
                        opFlag = false;
                        for j = 1:size(openList,2)
                            if min(openList{j}.cord == node.cord)
                                opFlag = true;
                                break;
                            end
                        end
                        if ~opFlag
                            openList{end+1} = node;
                        elseif openList{j}.cost > node.cost
                            openList{j} = node;
                        end
                    end
                end
            end
        end
    else
        disp('fail')
   
        break;
    end
end

pathList = {curNode};
while ~min(isnan(curNode.parent))
    for i = closeList
        if min(curNode.parent == i{1}.cord)
            curNode = i{1};
            pathList{end+1} = curNode;
            break;
        end
    end
end

nodeList = [openList,closeList];
opSize = size(openList,2);
for i = 1:size(nodeList,2)
    pathFlag = false;
    for j = pathList
        if min(nodeList{i}.cord == j{1}.cord)
            pathFlag = true;
        end
    end
    if i > opSize
        plotBlock(nodeList{i}.cord,[1,0.8,0]);
    else
        plotBlock(nodeList{i}.cord,'y');
    end
    if ~pathFlag
        [x,y] = turnCord(nodeList{i}.cord);
        text(x,y,num2str(nodeList{i}.cost,4),'HorizontalAlignment','center','Fontsize',14);
    end
end  
for i = pathList
    x = [i{1}.cord(1) i{1}.parent(1)];
    y = [i{1}.cord(2) i{1}.parent(2)];
    if ~isnan([x y])
        plot(x,y,'color',[102/255,204/255,255/255],'linewidth',3.6)
    end
end    

plotBlock(map.start,'r');
plotBlock(map.goal,'g');
plotGrid(map);

function plotGrid(map)
    lineWidth = 2;
    line_Xx = zeros(1,map.size(1));
    line_Xy = [-0.5,map.size(2)-0.5];
    line_Yy = zeros(1,map.size(2));
    line_Yx = [-0.5,map.size(1)-0.5];
    for i = 1:map.size(1)+1
        line_Xx(i) = i-1.5;
        plot([line_Xx(i) line_Xx(i)],line_Xy,'k','linewidth',lineWidth);
    end
    for i = 1:map.size(2)+1
        line_Yy(i) = i-1.5;
        plot(line_Yx,[line_Yy(i) line_Yy(i)],'k','linewidth',lineWidth);
    end
end

function plotBlock(cord,c)
    x = cord(1); y = cord(2);
    X = [x-0.5,x+0.5,x+0.5,x-0.5];
    Y = [y-0.5,y-0.5,y+0.5,y+0.5];
    fill(X,Y,c)
end

function [x,y] = turnCord(cord)
    x = cord(1); y = cord(2);
end