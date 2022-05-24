%%  Optimization Algorithm
start = 2.5;
loopCount = 100;
runTime = 1000;
args.weight = start;
args.debug = true;
turnFlag = false;
weightLog = zeros(1,2);
for i = 1:loopCount
    if ~turnFlag
        runTimeCache = runTime;
        runTime = AStarClassic(args);
        runTime = runTime.time;
        disp(['loopCount:' num2str(i) ' ' 'time:' num2str(runTime)]);
        if runTimeCache < runTime %优化过头了
            turnFlag = true;
            weightLog(2) = args.weight; %记录权重，准备二分法
            weightLog(1) = args.weight - 2;
        else
            args.weight = args.weight + 1;
            continue;
        end
    else
        args.weight = weightLog(1) + ((weightLog(2) - weightLog(1))*1/4);
        runTimeHigh = AStarClassic(args);
        runTimeHigh = runTimeHigh.time;
        args.weight = weightLog(1) + ((weightLog(2) - weightLog(1))*3/4);
        runTimeLow = AStarClassic(args);
        runTimeLow = runTimeLow.time;
        if runTimeHigh > runTimeLow %低侧较优
            weightLog(2) = (weightLog(2) + weightLog(1))/2;
            disp(['weight:' num2str(weightLog(2)) ' ' 'time:' num2str(runTimeLow)]);
        else                        %高侧较优
            weightLog(1) = (weightLog(2) + weightLog(1))/2;
            disp(['weight:' num2str(weightLog(1)) ' ' 'time:' num2str(runTimeHigh)]);
        end
    end
end