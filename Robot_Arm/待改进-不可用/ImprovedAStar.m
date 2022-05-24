function output = ImprovedAStar(args)

    global p560;
    mdl_puma560;
    if ~nargin
        args = NaN;
    end

    warning off
    
    global start goal;
    if isfield(args,'pathInfo')
        start = args.pathInfo.start;
        goal = args.pathInfo.goal;
        if isfield(args.pathInfo,'qStart')
            qStart = args.pathInfo.qStart;
        else
            qStart = p560.ikine6s(buildT(start));
        end
    else
        qStart = qn;
        start = p560.fkine(qn).t';
        goal = [-0.6,0.4,0.2];
    end

    startNode.cord = start;
    startNode.T = buildT(start);
    startNode.pose = qStart;
    startNode.parent = zeros(1,3)*NaN;
    startNode.cost = 0;

    goalNode.cord = goal;
    goalNode.T = buildT(goal);
    goalNode.pose = zeros(1,6)*NaN;
    goalNode.parent = zeros(1,3)*NaN;

    finNode = goalNode;

    global stepSize tolerance;
    stepSize = 0.8*pi/180;
    tolerance = 50/1000;
    weight = 4.5;

    hold on 

    global obList;
    if ~isfield(args,'ObstacleList')
        ball_1 = CreateObstacle(1,[0.3 -0.2 0],0.12);
        ball_2 = CreateObstacle(1,[-0.32 0 0.12],0.12);
        obList = [ball_1 ball_2];
    else
        obList = args.ObstacleList;
    end

    CreateSphere(0.08,start,[255 0 0]);
    CreateSphere(0.08,goal, [0 255 0]);

    disp('改进A*算法');tic;
    global pathList;

    pathList = {};
    roughPath = {};
    normPath = {};
    finePath = {};

    global openList closeList;
    curNode = startNode;
    openList = {curNode};
    closeList = {};

    breakFlag = false;
    curNode = startNode;
    p560.plot(startNode.pose);
    
    global curMode;
    curMode = 4;
    isColFlag = 0;
    style = 'r';
    while ~breakFlag
        disp(norm(curNode.cord - goal))
        cx = [curNode.cord(1) curNode.parent(1)];
        cy = [curNode.cord(2) curNode.parent(2)];
        cz = [curNode.cord(3) curNode.parent(3)];
        plot3(cx,cy,cz,style);
        if min(size(openList)) ~= 0
            minCost = 100;
            for i = 1:size(openList,2)
                if openList{i}.cost <= minCost
                    minCost = openList{i}.cost;
                    curNode = openList{i};
                end
            end
            if norm(curNode.cord - finNode.cord) < tolerance
                switch curMode
                case 1
                    finePath = buildPath(curNode,closeList);
                    breakFlag = true;
                    break;
                case 2
                    normPath = buildPath(curNode,closeList);
                    openList = normPath(ceil(size(normPath,2)/2));
                    closeList = {};
                    curMode = 1;
                    style = 'b'
                case 4
                    roughPath = buildPath(curNode,closeList);
                    openList = roughPath(ceil(size(roughPath,2)/3));
                    closeList = {};
                    curMode = 2;  
                    style = 'g'
                end
            else
                offsetList = [
                    -1 -1  0; -1  0 -1; -1  0  0; -1  0  1; -1  1  0;  0 -1 -1;
                     0 -1  0;  0 -1  1;  0  0 -1;  0  0  1;  0  1 -1;  0  1  0;
                     0  1  1;  1 -1  0;  1  0 -1;  1  0  0;  1  0  1;  1  1  0;];
                for i = 1:18
                    pose = curNode.pose + [offsetList(i,:) 0 0 0]*curMode*stepSize;
                    if isOutOfLim(pose)
                        continue
                    end
                    cord = p560.fkine(pose).t';
                    if isnan(findCord(closeList,cord)) && isnan(findCord(openList,cord))
                        TCache = p560.fkine(pose);
                        if isCollided(obList,p560,pose) <= 0
                            continue
                        else
                            nodeCache.cord = cord;
                            nodeCache.pose = pose;
                            nodeCache.T = TCache;
                            nodeCache.parent = curNode.cord;
                            cost = sum(abs(cord - start))/(1+weight) + sum(abs(cord - goal))*weight/(1+weight);
                            nodeCache.cost = Sigmoid(cost);
                            openList{end+1} = nodeCache;
                        end
                    end
                end
                closeList{end+1} = curNode;
                findNum = findCord(openList,curNode.cord);
                if ~isnan(findNum)
                    openList(findNum) = [];
                end
            end
        else
            disp('EMPTY');
            output.error = 'Empty open list';
            breakFlag = true;
            break;
        end
    end
    toc;
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
%%  检验从start节点到goal节点的路径是否发生碰撞
function output = isPathCollided(obList,robotArm,startT,goalT,length)
    %disp(startT);
    %disp(goalT);
    output = false;
    TList = ctraj(double(startT),double(goalT),length);
    try
        for i = 1:length
            q = robotArm.ikine6s(TList(:,:,i),zeros(1,6));
            if max(isnan(q))
                output = true;
                break;
            elseif isCollided(obList,robotArm,q) <= 0
                output = true;
                break;
            end
        end
    catch
        output = true;
    end
end
%%  Sigmoid函数
function output = Sigmoid(input)
    output = 1./(1+exp(-input));
end
%% 生成T
function T = buildT(cord,poseT)
    if nargin == 1
        poseT = eye(3);
    end
    T = [poseT,cord';[0 0 0 1]];
end