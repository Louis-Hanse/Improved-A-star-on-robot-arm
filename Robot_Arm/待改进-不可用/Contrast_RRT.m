% 2022.05.06 基本完成，但是在奇异位型附近时RRT会原地震荡
% 2022.05.05 生成随机坐标的时候(line 67)大概率会出现无法到达(unreachable)，需要另行判断
% 2022.05.04 RRT的生成方法错了，应该在某个方向上生成一个固定距离的点，而不是直接冲过去
function outputData = Contrast_RRT(args)

    global timeSum;
    timeSum = 0;

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
    goal = [0.08 0.12 0.16];                %目标点坐标

    global stepSize matSize;
    stepSize  = 20;             %单位mm
    matSize   = 2000/stepSize;  %每个轴都是[-1,1]，所以是2000mm
    tolerance = 0.08;

    %%  创建障碍信息，并将起点/终点可视化
    hold on

    ball_1 = CreateObstacle(1,[0.3 -0.2 0],0.12);
    %ball_2 = CreateObstacle(1,[0.48 -0.43 0.23],0.15);
    ball_2 = CreateObstacle(1,[-0.32 -0.08 0.2],0.12);
    %ball_3 = CreateObstacle(1,[0.3744 0.4043 0.1890],0.1);
    %wall_1 = CreateObstacle(2,0,[0 1 0 0.5]);

    global obList;
    obList = [ball_1 ball_2];

    startP = CreateSphere(0.08,start,[255 0 0]);
    goalP  = CreateSphere(0.08,goal, [0 255 0]);

    disp('RRT*');

    %hold off

    tic;

    startNode.cord = start;                     %每个节点以坐标作为id进行识别
    startNode.pose = qStart;                    %节点位姿
    startNode.T = p560.fkine(p560.getpos());    %代价
    startNode.parent = [NaN NaN NaN];           %起点parent为空，其余为上一级的坐标
    curNode = startNode;
    nodeList = {};
    nodeList{1} = curNode;

    while true
        testList = cell(1,20);
        testAngleList = zeros(1,20);
        % 随机生成20个节点
        for i = 1:20
            node.cord = zeros(1,3);
            for j = 1:3
                node.cord(j) = rand()*2 - 1;
            end
            node.T = [eye(3),[node.cord]';[0 0 0 1]];
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
            %disp(angle/pi*180)
            testAngleList(i) = angle;
            testList{i} = node;
        end
        %disp(size(testList))
        prepList = cell(1,4);
        for i = 1:4
            minAngle = pi;
            listSize = max(size(testList));
            angleFlag = 1;
            for j = 1:listSize
                if testAngleList(j) < minAngle
                    minAngle    = testAngleList(j);
                    angleFlag   = j;
                end
            end
            prepList{i} = testList{angleFlag};
            testList(angleFlag) = [];
            testAngleList(angleFlag) = [];
        end
        selNode = prepList{ceil(rand()*4)};
        %if min(size(selNode)) == 0
        %    disp('QAQ');
        %    continue
        %end
        selDist = 10;
        % 选择距拓展节点最近的节点
        nodeListSize = max(size(nodeList));
        for i = 1:nodeListSize
            if norm(selNode.cord - nodeList{i}.cord) < selDist
                cacheNode = nodeList{i};
            end
        end
        loopNum = ceil(norm(selNode.cord - cacheNode.cord)/0.05);
        try
            selVec = selNode.cord - cacheNode.cord;
            selVec = selVec/norm(selVec)*0.03;
            altSelNode.cord = cacheNode.cord + selVec;
            altSelNode.T = [eye(3),altSelNode.cord';[0 0 0 1]];
            altSelNode.pose = p560.ikine6s(altSelNode.T,curNode.pose);
            if max(isnan(altSelNode.pose)) == true
                continue
            end
            res = isPathCollided(obList,p560,altSelNode.T,cacheNode.T,loopNum);
        catch ME
            disp(ME)
            disp('SELNODE')
            disp(selNode)
            disp(mat2str(selNode.T))
            disp(cacheNode)
            disp(mat2str(cacheNode.T))
            continue
        end
        if ~res
            curNode = altSelNode;
            curNode.parent = cacheNode.cord;
            nodeList{end+1} = curNode;
            if norm(curNode.cord - goal) < tolerance
                finNode.cord = goal;
                finNode.T = [eye(3),goal';[0 0 0 1]];
                finNode.parent = curNode.cord;
                finNode.pose = p560.ikine6s(finNode.T,cacheNode.pose);
                toc;break;
            end
            cx = [altSelNode.cord(1) cacheNode.cord(1)];
            cy = [altSelNode.cord(2) cacheNode.cord(2)];
            cz = [altSelNode.cord(3) cacheNode.cord(3)];
            plot3(cx,cy,cz,'linewidth',2)
        end
    end
    
    disp('finished')

    hold off
    warning on
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

