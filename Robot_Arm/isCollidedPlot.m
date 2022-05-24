function isCollidedPlot(obList,robotArm)
    hold on
    q = robotArm.getpos();
    colInfo = isCollided(obList,robotArm,q,0.1,'debug');
    colSize = prod(size(colInfo));
    disp(size(colInfo))
    for i = 1:colSize
        if i == 3 || i == 4
            continue
        end
        %disp(colInfo{i})
        x = zeros(1,2); y = zeros(1,2); z = zeros(1,2);
        x(1) = colInfo{i}.closePoint(1); x(2) = colInfo{i}.obstacle.cord(1);
        y(1) = colInfo{i}.closePoint(2); y(2) = colInfo{i}.obstacle.cord(2);
        z(1) = colInfo{i}.closePoint(3); z(2) = colInfo{i}.obstacle.cord(3);
        plot3(x,y,z,'--or');
        midx = mean(x); midy = mean(y); midz = mean(z);
        text(midx,midy,midz,num2str(round(colInfo{i}.distance,3)),'Fontsize',14);
        if colInfo{i}.target
            x(2) = colInfo{i}.linkModel.cord(1,1);
            y(2) = colInfo{i}.linkModel.cord(2,1);
            z(2) = colInfo{i}.linkModel.cord(3,1);
            plot3(x,y,z,'--xb');
        end
        %text(2,8,'A Simple Plot','Color','red','FontSize',14)
    end
end