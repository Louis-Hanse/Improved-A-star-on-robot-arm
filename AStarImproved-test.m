%{
    Puma 560 [Unimation]:: 6 axis, RRRRRR, stdDH, slowRNE                        
 +---+-----------+-----------+-----------+-----------+-----------+
 | j |     theta |         d |         a |     alpha |    offset |
 +---+-----------+-----------+-----------+-----------+-----------+
 |  1|         q1|          0|          0|     1.5708|          0|
 |  2|         q2|          0|     0.4318|          0|          0|
 |  3|         q3|    0.15005|     0.0203|    -1.5708|          0|
 |  4|         q4|     0.4318|          0|     1.5708|          0|
 |  5|         q5|          0|          0|    -1.5708|          0|
 |  6|         q6|          0|          0|          0|          0|
 +---+-----------+-----------+-----------+-----------+-----------+
%}
function outputData = AStarImproved(args)
    
    %clc;
    clear;
    warning off

    global p560;
    mdl_puma560 %调用PUMA560机械臂作为研究对象

    if ~nargin
        args.weight = 4;
    end

    %%  参数设置
    qStart = [0 pi/4 pi 0 pi/4 0];          %起始位姿
    p560.plot(qStart,'jointlen',1.5)  
    global start goal; 
    start = p560.fkine(p560.getpos()).t;    %起始点坐标
    start = start.';
    goal = [-0.6 0.4 0.2];                  %目标点坐标
    goalPos = eye(3);                       %目标点执行器姿态
    goalT = [goalPos goal';[0 0 0 1]];      %目标点位姿  
    goalNode.cord = goal;   
    goalNode.T = goalT;                

    global stepSize matSize;
    stepSize  = 20;             %单位mm
    matSize   = 2000/stepSize;  %每个轴都是[-1,1]，所以是2000mm
    tolerance = norm(stepSize/1000*ones(1,3));

    %%  创建障碍信息，并将起点/终点可视化
    hold on

    ball_1 = CreateObstacle(1,[0.3 -0.2 0],0.12);
    %ball_2 = CreateObstacle(1,[0.48 -0.43 0.23],0.15);
    ball_2 = CreateObstacle(1,[-0.32 -0.08 0.2],0.12);
    %wall_1 = CreateObstacle(2,0,[0 1 0 0.5]);

    global obList;
    obList = [ball_1 ball_2];

    CreateSphere(0.08,start,[255 0 0]);
    CreateSphere(0.08,goal, [0 255 0]);

    disp('改进A*算法');

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

    %%  定义变量
    global openList closeList;tic;

    openList = {};
    closeList = {};
    pathList = {};
    global catchList;
    catchList = {};
    startNode.cord = start;             %每个节点以坐标作为id进行识别
    startNode.pose = qStart;            %节点位姿
    startNode.cost = 0;                 %代价
    startNode.parent = [NaN NaN NaN];   %起点parent为空，其余为上一级的坐标
    startNode.T = p560.A(1:6,qn);
    openList{1} = startNode;
    tarNode = startNode;
    flag = false;
    jumpFlag = false;
    loopCount = 1;
    loopCheck = 50;

    while true
        logFile = fopen('log.txt','w');
        fprintf(logFile,'%g\n',tarNode.cord);
        fprintf(logFile,'%f\n',norm(tarNode.cord - goal));
        fprintf(logFile,'%d',loopCount);
        fclose(logFile);
        if min(size(openList)) ~= 0
            minCost = 1;
            % 找到代价最低的点
            for i = openList
                if minCost >= i{1}.cost
                    minCost = i{1}.cost;
                    tarNode = i{1};
                end
            end
            if norm(tarNode.cord - goal) < tolerance
                disp('Path found! Tracing back...');
                flag = true;
                break;
            else
                openList_expand(tarNode,args.weight);
                moveToCloseList(tarNode);
            end
        else
            disp('ERR:Open list is empty!');
            break;
        end
        loopCount = loopCount + 1;
        %{
        if loopCount >= loopCheck
            loopCheckOffset = Sigmoid(loopCount*0.01-5)*(-90) + 100;
            if loopCheckOffset < 10
                loopCheckOffset = 10;
            else
                loopCheckOffset = round(loopCheckOffset);
            end
            loopCheck = loopCheck + loopCheckOffset;
            goalNode.pose = p560.ikine6s(goalT,tarNode.pose);
            goalNode.T = p560.A(1:6,goalNode.pose);
            if ~isPathCollided(obList,p560,tarNode,goalNode,50)
                flag = true;
                jumpFlag = true;
                if loopCount > 900
                    break;
                end
            end
        end
        %}
    end

    disp(['Total loop:',num2str(loopCount)]);
    %%  找到路径后向后跟踪并优化
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
        posList = zeros(pathSize,6);
        for i = pathSize:-1:1
            pathList{i} = node;
            posList(i,:) = node.pose;
            length = length + norm(node.cord - node.parent);
            node = closeList{findCord(closeList,node.parent)};
            angle = angle + abs(node.pose - posList(i));
        end
        if jumpFlag
            qC = p560.jtraj(pathList{end}.T,goalNode.T,160);
            posList = [posList;qC];
            pathSize = pathSize + 160;
        end
        disp('Path established!');
        %disp(['Path pose:',mat2str(angle/pi*360)]);
        outputData.time = toc;pause;
        outputData.length = length;
        outputData.angle = angle;
        %disp(['length:',num2str(length*1000),'mm']);
        disp(['time:',num2str(outputData.time),'s']);
        for i = 1:pathSize
            p560.plot(posList(i,:));
            curCord = p560.A(1:6,posList(i,:)).t;
            x = curCord(1); y = curCord(2); z = curCord(3);
            plot3(x,y,z,'r.');
        end
    end

    hold off
    warning on
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
    global p560 obList start goal; 
    global stepSize openList closeList;
    pose = [];
    cord = [];
    TCache = [];
    if nargin == 1
        weight = 3.5;
    end
    try
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
                    nodeCache.T = TCache;
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

%%  检验从start节点到goal节点的路径是否发生碰撞
function output = isPathCollided(obList,robotArm,startNode,goalNode,length)
    output = false;
    qList = robotArm.jtraj(startNode.T,goalNode.T,length);
    for i = 1:length
        if isCollided(obList,robotArm,qList(i,:)) <= 0
            output = true;
            break;
        end
    end
end

%%  Sigmoid函数
function output = Sigmoid(input)
    output = 1./(1+exp(-input));
end