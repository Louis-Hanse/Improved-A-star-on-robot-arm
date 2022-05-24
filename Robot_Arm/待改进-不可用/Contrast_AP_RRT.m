%{
    start
while
    cord = fkine
    if isCollided == true
        ?end
    else
        for i = 1:18
            sumPotential = calc
        end
        node = getMinPotential
        if node == fin
            if node == trueFin
                STOP
            else
                fin = trueFin
            end
        else
            if node == CacheNode
                % Enter ARRT
                q_start = node
                q_s_target = getTar
                q_new = vec/|vec|*lenth
                q_new = falseFin
        end
end
%}
function outputData = Contrast_AP_RRT(input)
    warning off

    if ~nargin
        args.weight = 3;
        args.debug = false;
    end

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
    tolerance = 0.08;

    %%  创建障碍信息，并将起点/终点可视化
    hold on

    %ball_1 = CreateObstacle(1,[0.3 -0.2 0],0.09);
    %ball_2 = CreateObstacle(1,[0.48 -0.43 0.23],0.15);
    ball_2 = CreateObstacle(1,[-0.32 -0.08 0.2],0.09);
    %ball_3 = CreateObstacle(1,[0.3744 0.4043 0.1890],0.1);
    %wall_1 = CreateObstacle(2,0,[0 1 0 0.5]);

    global obList;
    obList = [ ball_2];

    startP = CreateSphere(0.08,start,[255 0 0]);
    goalP  = CreateSphere(0.08,goal, [0 255 0]);

    disp('AP+ARRT');

    %hold off

    startNode.cord = start;                     %每个节点以坐标作为id进行识别
    startNode.pose = qStart;                    %节点位姿
    startNode.T = p560.fkine(p560.getpos());    %代价
    curNode = startNode;
    nodeList = {};
    nodeList{1} = curNode;    
    finCord = goal;
    tic;

    while true
        if isCollided(obList,p560,curNode.pose) < 0
            disp('ERR:Collided');
            disp(isCollided(obList,p560,curNode.pose));
            break;
        end
        % 拓展节点并计算势能
        potList = zeros(1,18)*NaN;
        selList = cell(1,18);
        offsetList = [  
            1 0 0;  0 1 0;  0 0 1;  -1 0 0;  0 -1 0;  0 0 -1;
            0 1 1;  1 0 1;  1 1 0; 0 -1 -1; -1 0 -1; -1 -1 0;
            0 -1 1; -1 0 1; -1 1 0;  0 1 -1;  1 0 -1;  1 -1 0;]*0.04;
        for i = 1:18
            expCord = curNode.cord + offsetList(i);
            if isnan(findCord(nodeList,expCord))
                Tcache = [eye(3),expCord';[0 0 0 1]];
                pose = p560.ikine6s(Tcache,curNode.pose);
                if max(isnan(pose)) == true
                    continue
                else
                    potList(i) = calcPotential(expCord,finCord);
                    selNode.cord = expCord;
                    selNode.T = [eye(3),expCord';[0 0 0 1]];
                    selNode.pose = p560.ikine6s(selNode.T,curNode.pose);
                    selList{i} = selNode;
                end
            end
        end

        % 去掉无法到达的点
        selListSize = size(selList);
        selListSize = selListSize(2);
        i = 1;
        while true
            if isnan(potList(i)) || isCollided(obList,p560,selList{i}.pose) < 0
                selList(i) = [];
                potList(i) = [];
            else
                i = i + 1;
            end
            selListSize = size(selList);
            selListSize = selListSize(2);
            if i > selListSize
                break
            end
        end
        x = curNode.cord(1); y = curNode.cord(2); z = curNode.cord(3);
        plot3(x,y,z,'rx','Markersize',10)
        minPot = max(potList) + 1;
        minPotNode = selList{1};
        nodeMarkList = [];
        
        % 找出势能最小的点
        for i = 1:selListSize
            if potList(i) < minPot
                minPot = potList(i);
                minPotNode = selList{1};
            end

        end
        nodeList{end+1} = minPotNode;
        if norm(minPotNode.cord - finCord) < tolerance
            if norm(finCord - goal) < 0.02
                goalNode.cord = goal;
                goalNode.T = [eye(3),goal';[0 0 0 1]];
                goalNode.pose = p560.ikine6s(goalNode.T,minPotNode.pose);
                nodeList{end+1} = goalNode;
                break
            else
                finCord = goal;
            end
        else
            if norm(minPotNode.cord - curNode.cord) < 0.02
                % 震荡,进入ARRT
                testList = cell(1,20);
                testAngleList = zeros(1,20);
                % 随机生成点
                for i = 1:20
                    while true
                        node.pose = genRandomPose();
                        node.T = p560.fkine(pose);
                        node.cord = node.T.t';
                        node.T = double(node.T);
                        if isCollided(obList,p560,node.pose) > 0
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
                            testAngleList(i) = angle;
                            testList{i} = node;
                            break
                        end
                    end                    
                end
                % 删去比较离谱的后几个节点
                for i = 1:10
                    listSize = size(testList);
                    listSize = listSize(2);
                    maxFlag = 1;
                    maxAngle = -1;
                    for j = 1:listSize
                        if maxAngle < testAngleList(j)
                            maxAngle = testAngleList(j);
                            maxflag = j;
                        end
                    end
                    testAngleList(maxFlag) = [];
                    testList(maxFlag) = [];
                end
                % 轮盘赌，随机选出一个节点作为q_target
                tarNode = testList{ceil(rand()*10)};
                relVec = tarNode.cord - minPotNode.cord;
                relVec = relVec/norm(relVec)*0.06;
                finCord = minPotNode.cord + relVec;
            end
            curNode = minPotNode;
        end
    end

    hold on
    nodeListSize = size(nodeList);
    nodeListSize = nodeListSize(2);
    for i = 1:nodeListSize
        p560.plot(nodeList{i}.pose);
        if i ~= nodeListSize
            cx = [nodeList{i}.cord(1) nodeList{i+1}.cord(1)];
            cy = [nodeList{i}.cord(2) nodeList{i+1}.cord(2)];
            cz = [nodeList{i}.cord(3) nodeList{i+1}.cord(3)];
            plot3(cx,cy,cz,'linewidth',2);
        end
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

function output = calcPotential(cord,finCord)
    global obList p560;
    ka = 4;
    kr = 100;
    l0 = 0.12;
    U_att = 0.5*ka * norm(cord-finCord)^2;
    pose = p560.ikine6s([eye(3),cord';[0 0 0 1]]);
    distList = isCollided(obList,p560,pose,0.075,'complete');
    U_irList = zeros(1,prod(size(distList)));
    for i = 1:prod(size(distList))
        if distList(i) <= l0
            U_irList(i) = 0.5*kr * ((1/distList(i)) - (1/l0))^2;
        end
    end
    U_ir = sum(sum(U_irList));
    output = U_att + U_ir;
end