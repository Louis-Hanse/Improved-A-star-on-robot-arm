%{  
    Function:   检测机械臂是否与障碍物碰撞，连杆简化为两端为半球的圆柱
    Input:      ObstacleList:   障碍物信息
                robotArm：      机械臂的SerialLink
                safeDistance:   安全裕量，单位与绘图窗口相同为m
                mode:           'simplified'    返回最小值
                                'complete'      返回完整矩阵
                                'debug'         测试用
    RetVal:     去除安全裕量后的距离矩阵，若返回值中存在<=0则为已经碰撞
%}
function distance = isCollided(ObstacleList,robotArm,q,safeDistance,mode)
    %%  初始化
    linkR = 0.01;    %连杆尺寸
    if nargin <= 4
        mode = 'simplified';
    end

    if nargin <= 3
        safeDistance = 0.1 + linkR; %设置默认安全裕量
    else
        safeDistance = safeDistance + linkR;
    end

    if nargin == 2
        q = robotArm.getpos();
    end

    %%  建立机械臂各个连杆模型
    cordArr = getCord(robotArm,q); %3个连杆共4个节点的[x;y;z]坐标->3x4矩阵
    %{
    测试函数，标记节点
    for i = 1:4
        arrCache = cordArr(1:3,i:i)';
        CreateSphere(0.08,arrCache,[255 255 255]);
    end
    %}
    armSize = size(cordArr);
    armSize = armSize(2)-1; %节点数-1 = 连杆数
    linkModel = cell(1,armSize);
    for i = 1:armSize
        arr = zeros(3,2);
        arr(:,1) = cordArr(:,i);
        arr(:,2) = cordArr(:,i+1);
        linkModel{i}.cord = arr; %3x2矩阵
        linkModel{i}.eq = arr(:,2) - arr(:,1);
    end
    % 距离矩阵预分配内存
    distanceSize = size(ObstacleList);
    distanceSize = distanceSize(2);
    distance = zeros(distanceSize,armSize); %obList列 x armSize行
    % disp(['Obstacles:',num2str(distanceSize),'Links:',num2str(armSize)]);  

    if strcmp(mode,'debug')
        distance = cell(distanceSize,armSize);
    end
    %%  检测某个连杆是否与障碍物碰撞
    count = 1;
    for i = ObstacleList
        for j = linkModel %遍历各个连杆
            if strcmp(mode,'debug')
                dist = [];
                dist.obstacle = i;
                dist.linkModel = j{1};
                dist.target = 0;
            end
            % 球体障碍物
            if strcmp(i.type,'sphere')
                % 计算平面偏置d
                d = i.cord * j{1}.eq;
                % 计算平面与连杆直线相交点参数t
                x = j{1}.cord(1,1); y = j{1}.cord(2,1); z = j{1}.cord(3,1);
                t = (d + [x y z]*j{1}.eq) / sum(j{1}.eq.^2);
                if t>0 && t<1 %最近点在连杆内，直接计算
                    cx = j{1}.eq(1)*t+x;
                    cy = j{1}.eq(2)*t+y;
                    cz = j{1}.eq(3)*t+z;
                    distanceCache = norm([cx cy cz] - i.cord) - safeDistance;
                else %最近点在连杆外，计算连杆两端点取最小值
                    d1 = norm(j{1}.cord(:,1)' - i.cord) - safeDistance;
                    d2 = norm(j{1}.cord(:,2)' - i.cord) - safeDistance;
                    distanceCache = min([d1 d2]);
                    if strcmp(mode,'debug')
                        dist.target = 1;
                    end
                end
                distanceCache = distanceCache - i.arg;
                if strcmp(mode,'debug')
                    dist.distance = distanceCache;
                    dist.closePoint = [j{1}.eq(1)*t+x j{1}.eq(2)*t+y j{1}.eq(3)*t+z];
                    distance{count} = dist;
                else
                    distance(count) = distanceCache;
                end

            % 长方体（壁）障碍物
            elseif strcmp(i.type,'wall')
                %disp('WIP');
                distance(count) = 0.1;
            end
            count = count + 1;
        end
    end

    for i = 1:size(distance,1)
        distance(i,end) = distance(i,end)-0.05;
    end
    %%  格式化输出
    if strcmp(mode,'simplified')
        distance = min(min(distance));
    end
    
end

%{
    debug模式备忘
    obstacle        障碍物信息
    distance        距离
    closePoint      最近点坐标
    linkModel       连杆直线的三个分母和d
%}