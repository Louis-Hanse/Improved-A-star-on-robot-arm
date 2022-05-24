% 绘制各个节点
function plotResult(pathList,styleL,styleP)
    start = pathList{1}.cord;
    goal = pathList{end}.cord;
    listSize = max(size(pathList));
    figure(2);
    hold on
    x = zeros(1,listSize+1);
    y = zeros(1,listSize+1);
    for i = 1:listSize
        distanceS = norm(pathList{i}.cord - start);
        distanceG = norm(pathList{i}.cord - goal);
        x(i) = (i-1)/listSize;
        y(i) = distanceS/(distanceG + distanceS);
        plot(x(i),y(i),styleP,'Markersize',25)
    end
    plot(1,1,styleP,'Markersize',20)
    x(end) = 1;
    y(end) = 1;
    plot(x,y,styleL,'linewidth',3);
end