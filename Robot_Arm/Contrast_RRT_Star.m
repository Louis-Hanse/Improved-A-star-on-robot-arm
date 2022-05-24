function output = Contrast_RRT_Star(args)
    
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

    goalNode.cord = goal;
    goalNode.T = buildT(goal);
    goalNode.pose = zeros(1,6)*NaN;
    goalNode.parent = zeros(1,3)*NaN;

    global stepSize tolerance;
    stepSize = 48/1000;
    tolerance = 96/1000;

    hold on 

    global obList;
    if ~isfield(args,'ObstacleList')
        ball_1 = CreateObstacle(1,[0.3 -0.2 0],0.12);
        %ball_2 = CreateObstacle(1,[0.48 -0.43 0.23],0.15);
        ball_2 = CreateObstacle(1,[-0.32 0.3 0.12],0.12);
        %ball_3 = CreateObstacle(1,[0.3744 0.4043 0.1890],0.1);
        %wall_1 = CreateObstacle(2,0,[0 1 0 0.5]);
        obList = [ball_1 ball_2];
    else
        obList = args.ObstacleList;
    end
    CreateSphere(0.08,start,[255 0 0]);
    CreateSphere(0.08,goal, [0 255 0]);

    disp('改进RRT算法');tic;
    global pathList nodeList;
    pathList = {startNode};
    nodeList = {startNode};
    breakFlag = false;
    curNode = startNode;
    p560.plot(startNode.pose);

    global NanNode;
    NanNode.cord = zeros(1,3)*NaN;
    NanNode.T = zeros(4)*NaN;
    NanNode.pose = zeros(1,6)*NaN;
    NanNode.parent = zeros(1,3)*NaN;

    while breakFlag == false
        randList = cell(1,20);
        angleList = zeros(1,20);
        for i = 1:20
            node = NanNode;
            node.cord = rand(1,3)*2-1;
            node.T = buildT(node.cord);
            %计算角度值
            vec1 = goal - start;
            vec2 = node.cord - start;
            isAcute = dot(vec1,vec2);
            if isAcute == 0
                angle = pi/2;
            else
                isAcute = isAcute/abs(isAcute);    
                angle = asin(norm(cross(vec1,vec2))/(norm(vec1)*norm(vec2)));
                if isAcute == -1
                    angle = angle + pi/2;
                end
            end
            angleList(i) = angle;
            randList{i} = node;
        end
        %挑选最优的5个进入备选名单
        prepList = cell(1,5);
        for i = 1:5
            minAnle = min(angleList);
            listSize = size(randList,2);
            for j = 1:listSize
                if angleList(j) == minAnle
                    break;
                end
            end
            prepList{i} = randList{j};
            randList(j) = [];
            angleList(j) = [];
        end
        selNode = prepList{ceil(rand()*5)};
        %找出距选择节点距离最短的节点
        distList = ones(1,size(nodeList,2))*4;
        for i = 1:size(nodeList,2)
            distList(i) = norm(nodeList{i}.cord - selNode.cord);
        end
        minDist = min(distList);
        for i = 1:size(nodeList,2)
            if distList(i) == minDist
                break;
            end
        end
        selFlag = i;
        selVec = selNode.cord - nodeList{selFlag}.cord;
        selVec = selVec/norm(selVec)*stepSize;
        altNode.cord = nodeList{selFlag}.cord + selVec;
        altNode.T = buildT(altNode.cord);
        try
            altNode.pose = p560.ikine6s(altNode.T);
            if max(isnan(altNode.pose))
                res = -1;
            else
                res = isCollided(obList,p560,altNode.pose);
            end
        catch ME
            res = -1;
            continue
        end
        if res < 0
            continue
        else
            curNode = altNode;
            curNode.parent = nodeList{selFlag}.cord;
            nodeList{end+1} = curNode;
            if norm(curNode.cord - goal) < tolerance
                finNode.cord = goal;
                finNode.T = [eye(3),goal';[0 0 0 1]];
                finNode.parent = curNode.cord;
                finNode.pose = p560.ikine6s(finNode.T,curNode.pose);
                output.time = toc;breakFlag = true;
            end
            cx = [altNode.cord(1) nodeList{selFlag}.cord(1)];
            cy = [altNode.cord(2) nodeList{selFlag}.cord(2)];
            cz = [altNode.cord(3) nodeList{selFlag}.cord(3)];
            plot3(cx,cy,cz,'linewidth',2,'Color',[100/255,149/255,237/255,0.4]);
        end
    end

    curNode = finNode;
    cacheList = {finNode};
    while ~max(isnan(curNode.parent))
        curNode = nodeList{findCord(nodeList,curNode.parent)};
        cacheList{end+1} = curNode;
    end
    dist = 0;
    for i = 1:size(cacheList,2)-1
        cx = [cacheList{i}.cord(1) cacheList{i+1}.cord(1)];
        cy = [cacheList{i}.cord(2) cacheList{i+1}.cord(2)];
        cz = [cacheList{i}.cord(3) cacheList{i+1}.cord(3)];
        plot3(cx,cy,cz,'linewidth',2,'Color',[1 0 0]);
        dist = dist + norm(cacheList{i}.cord - cacheList{i+1}.cord);
    end
    output.length = dist;

    for i = size(cacheList,2):-1:1
        pathList{end+1} = cacheList{i};
    end
    output.path = pathList;

    warning on

end

function T = buildT(cord,poseT)
    if nargin == 1
        poseT = eye(3);
    end
    T = [poseT,cord';[0 0 0 1]];
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