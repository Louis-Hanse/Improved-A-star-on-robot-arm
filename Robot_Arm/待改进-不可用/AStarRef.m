warning off

args.weight = 3.5;
args.debug = false;

global p560;
mdl_puma560 %调用PUMA560机械臂作为研究对象
qStart = [0 pi/4 pi 0 pi/4 0];                  %起始位姿
p560.plot(qStart,'jointlen',1.5)  
%p560.plot([1 1 1 1 1 1],'jointlen',1.5)        %测试位姿
global start goal; 
start = p560.fkine(p560.getpos()).t;            %起始点坐标
start = start.';
goal = [-0.6 0.4 0.2];                %目标点坐标

global stepSize matSize;
stepSize  = 20;             %单位mm
matSize   = 2000/stepSize;  %每个轴都是[-1,1]，所以是2000mm
tolerance = 0.04;

%%  创建障碍信息，并将起点/终点可视化
hold on

ballArgs.arg = 0.15;
ballArgs.testMode = true;
ball_1 = CreateObstacle(1,[0.3 -0.2 0],ballArgs);
%ball_2 = CreateObstacle(1,[0.48 -0.43 0.23],0.15);
ball_2 = CreateObstacle(1,[-0.32 -0.08 0.2],ballArgs);
%wall_1 = CreateObstacle(2,0,[0 1 0 0.5]);

global obList;
obList = [ball_1 ball_2];

startP = CreateSphere(0.08,start,[255 0 0]);
goalP  = CreateSphere(0.08,goal, [0 255 0]);

disp('start');

%hold off

%{
完整的A*算法描述如下：
* 初始化open_set和close_set；
* 将起点加入open_set中，并设置优先级为0（优先级最高）；
* 如果open_set不为空，则从open_set中选取优先级最高的节点n：
    * 如果节点n为终点，则：
        * 从终点开始逐步追踪parent节点，一直达到起点；
        * 返回找到的结果路径，算法结束；
    * 如果节点n不是终点，则：
        * 将节点n从open_set中删除，并加入close_set中；
        * 遍历节点n所有的邻近节点：
            * 如果邻近节点m在close_set中，则：
                * 跳过，选取下一个邻近节点
            * 如果邻近节点m也不在open_set中，则：
                * 设置节点m的parent为节点n
                * 计算节点m的优先级
                * 将节点m加入open_set中    
%}

%%  初始化
global openList closeList;
tic;

openList = {};
closeList = {};
pathList = {};
global catchList;
catchList = {};
startNode.cord = start;             %每个节点以坐标作为id进行识别
startNode.pose = qStart;            %节点位姿
startNode.cost = 0;                 %代价
startNode.parent = [NaN NaN NaN];   %起点parent为空，其余为上一级的坐标
openList{1} = startNode;
tarNode = startNode;
flag = false;
loopCount = 1;
costP = 0;

%%  算法主要部分
while true
    %disp(['Cord:' mat2str(round(tarNode.cord,3))]);
    %disp(['Dist:' num2str(norm(tarNode.cord - goal))]);
    logFile = fopen('log.txt','w');
    fprintf(logFile,'%g\n',tarNode.cord);
    %fprintf(logFile,'%f\n',norm(tarNode.cord - goal));
    costG = sum(abs(tarNode.cord - start));
    costP = tarNode.pose(1:3)*[2;2;1];
    fprintf(logFile,'%f\n',(costP/costG));
    fprintf(logFile,'%d',loopCount);
    fclose(logFile);
    loopCount = loopCount + 1;
    cacheNode = tarNode;
    if min(size(openList)) ~= 0
        minCost = 1;
        % 找到代价最低的点
        for i = openList
            if minCost >= i{1}.cost
                minCost = i{1}.cost;
                tarNode = i{1};
            end
        end
        disp(mat2str((cacheNode.pose - tarNode.pose)/pi*180));
        if norm(tarNode.cord - goal) < tolerance
            disp('Path found! Tracing back...');
            flag = true;
            break;
        else
            moveToCloseList(tarNode);
            openList_expand(tarNode,args.weight);
        end
    else
        disp('ERR:Open list is empty!');
        break;
    end
end

%%  反向追踪
if flag
    node = tarNode;
    pathSize = 1;
    while true
        node = closeList{findCord(closeList,node.parent)};
        if max(isnan(node.parent))
            break;
        end
        pathSize = pathSize + 1;
    end
    node = tarNode;
    length = 0;
    angle = zeros(1,6);
    for i = pathSize:-1:1
        pathList{i} = node;
        angleCache = node.pose;
        length = length + norm(node.cord - node.parent);
        node = closeList{findCord(closeList,node.parent)};
        angle = angle + abs(node.pose - angleCache);
    end
    %{
    pose = pathList{end}.pose(1:3)';
    goalNode.cord = goal;
    goalNode.pose = p560.ikine6s([eye(3),pose;[0 0 0 1]]);
    goalNode.cost = 0;
    goalNode.parent = pathList{end}.cord;
    pathList{end+1} = goalNode;
    %}
    disp('Path established!');
    outputData.time = toc;
    outputData.length = length;
    outputData.angle = angle;
    outputData.pathList = pathList;
    disp(['Length:' num2str(length)]);
    if ~args.debug
        pause;
    else 
        return;
    end
    for i = pathList
        p560.plot(i{1}.pose);
        x = i{1}.cord(1);
        y = i{1}.cord(2);
        z = i{1}.cord(3);
        %plot3(x,y,z,'r.');
    end
end

hold off
warning on

%%
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

%%  从open list删除节点并移动进close list
function moveToCloseList(node)
global openList closeList;
i = findCord(openList,node.cord);
if isnan(i) %节点不存在于open list中
    disp("ERR:Node don't exist!");
else 
    openList(i) = [];
end
if isnan(findCord(closeList,node.cord))
    closeList{end+1} = node;
end
end

%%  从当前节点拓展open list
function openList_expand(node,weight)
    global catchNode catchList;
    pose = [];
    cord = [];
    TCache = [];
    if nargin == 1
        weight = 3.5;
    end
    try
        global p560 obList start goal; 
        global stepSize openList closeList;
        dist = stepSize/1000;
        offsetList = [
            1 0 0;  0 1 0;  0 0 1;  -1 0 0;  0 -1 0;  0 0 -1;
            0 1 1;  1 0 1;  1 1 0; 0 -1 -1; -1 0 -1; -1 -1 0;
            0 -1 1; -1 0 1; -1 1 0;  0 1 -1;  1 0 -1;  1 -1 0;]*dist;

        for i = 1:18
            cord = node.cord + offsetList(i,:);
            if isnan(findCord(closeList,cord)) && isnan(findCord(openList,cord)) 
                % 不存在于close list和open list中
                TCache = [eye(3),cord';[0 0 0 1]];
                pose = p560.ikine6s(TCache,node.pose);
                isOutOfLim = false;
                for j = 1:3
                    qLim = p560.qlim(j,:);
                    if pose(j) <= min(qLim)
                        while true
                            if pose(j) <= min(qLim) && pose(j)+2*pi >= max(qLim)
                                isOutOfLim = true;
                                break;
                            elseif pose(j) > min(qLim) && pose(j) < max(qLim)
                                break;
                            else
                                pose(j) = pose(j) + 2*pi;
                            end
                        end
                    elseif pose(j) >= max(qLim)
                        while true
                            if pose(j) >= max(qLim) && pose(j)-2*pi <= min(qLim)
                                isOutOfLim = true;
                                break;
                            elseif pose(j) > min(qLim) && pose(j) < max(qLim)
                                break;
                            else
                                pose(j) = pose(j) - 2*pi;
                            end
                        end
                    end
                end
                pose(4:6) = [0 0 0];
                if isCollided(obList,p560,pose) >= 0 && ~max(isnan(pose)) && ~isOutOfLim 
                    % 没有与障碍物发生碰撞且位置可到达
                    nodeCache.cord = cord;
                    nodeCache.parent = node.cord;
                    nodeCache.pose = pose;
                    cost = sum(abs(cord - start))/(1+weight) + sum(abs(cord - goal))*weight/(1+weight);
                    nodeCache.cost = 1/(1+exp(-1*cost)); %用sigmoid函数将代价限制在[0,1)
                    openList{end+1} = nodeCache;
                end
            end
        end
    catch
        catchNode.pose = pose;
        catchNode.cord = cord;
        catchNode.TCache = TCache;
        catchList{end+1} = catchNode;
    end
end