clc;clear;
mdl_puma560

accSize = 10000;
thetaList = zeros(6,accSize);
for i = 1:6
    theta_i = rand(1,accSize)*(p560.qlim(i,2)-p560.qlim(i,1)) + p560.qlim(i,1);
    thetaList(i,:) = theta_i;
end

hold on
view(3);
Wait_Title = waitbar(0,'Calculating...');
pointList = zeros(accSize,3);
for i = 1:accSize
    cord = p560.fkine(thetaList(:,i)').t;
    x = cord(1); y = cord(2); z = cord(3);
    pointList(i,:) = [x y z];
    plot3(x,y,z,'r.','Markersize',5);
    Display_Data = num2str(roundn(i/accSize,-3));
                     % Calculate percentage
    Display_Str = ['Progress: ',Display_Data,'%'];
                     % Show Calculate State
    waitbar(i/accSize,Wait_Title,Display_Str)
                     % Progress bar dynamic display   
end

close(Wait_Title);   % Close Progress bar window