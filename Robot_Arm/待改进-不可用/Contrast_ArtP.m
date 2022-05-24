function output = Contrast_ArtP(args)

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

    
end