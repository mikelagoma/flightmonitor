%% Configuation Section
numClusters     = 4;              % number of Sensor clusters
calibrationTime = 10;             % seconds
launchAngle  = 15;                % degrees

LaunchThreshMinAccel   = 1;       % m/s/s;
LaunchThreshNumSensors = 3;
isPrintSensor = 0;
isPrintRaw = 0;
isPrintUvectors = 0;

%%Initialization
format short;
clear('res');
numSensors = numClusters * 3;

%Create serial object
if exist('uart')            %reset Serial Object
    fclose(uart);
    clear uart;
end
uart=serial('/dev/tty.usbserial-A900XZDP','BaudRate',9600);
fopen(uart);                %Open serial and start timer

%Data collection
curAccelRaw(numClusters,3) = zeros();
oldAccelRaw(numClusters,3) = zeros();
difAccelRaw(numClusters,3) = zeros();
curAccelUvector(numClusters,3) = zeros();
oldAccelUvector(numClusters,3) = zeros();
difAccelUvector(numClusters,3) = zeros();
difAccelMag(numClusters) = zeros();
curAccelMag(numClusters) = zeros();
oldAccelMag(numClusters) = zeros();

disp('Calibrating Projectile');
idx     =   1;
errors  =   0;
rawObs(1,numSensors)                  = zeros();
launch_accel_raw(numClusters,1,3)     = zeros();
launch_accel_norm(numClusters,1,4)    = zeros();
tic; timeLapsed = 0;

calib_A(numClusters,1,2) = zeros();
while (timeLapsed < calibrationTime)
    idx = idx+1;
    reading=fscanf(uart,'%s');          %Serial read
    
    if(length(reading)==36)
        for j=1:numSensors
            %Split arduino string of all readings to 3 char single readings
            %Convert 3 char raw reading string to double then to G's
            readingstr = char(reading((j-1)*3+1:(j-1)*3+3));
            rawObs(idx ,j) = (str2double(readingstr)-512)/102.3*9.81;
        end
    else
        continue;
    end
    
    oldAccelRaw     = curAccelRaw;         
    oldAccelMag     = curAccelMag;
    oldAccelUvector = curAccelUvector;
       
    %due to orientation of accelerometers
    curAccelRaw(1,:) = [  rawObs(idx,2)    rawObs(idx,1)  rawObs(idx,3) ]; % left wing
    curAccelRaw(2,:) = [  rawObs(idx,5)    rawObs(idx,4)  rawObs(idx,6) ]; % right wing
    curAccelRaw(3,:) = [  rawObs(idx,7)   -1*rawObs(idx,8) rawObs(idx,9) ]; % nose
    curAccelRaw(4,:) = [ -1*rawObs(idx,10)   rawObs(idx,11) rawObs(idx,12) ];     % back sensor
    for idxSensor = 1:numClusters
        [curAccelMag(idxSensor) curAccelUvector(idxSensor,:)] = NormalizeVector(curAccelRaw(idxSensor,:));
        Ax = atan(curAccelRaw(idxSensor,1)/sqrt(curAccelRaw(idxSensor,2)^2 + curAccelRaw(idxSensor,3)^2));
        Ay = atan(curAccelRaw(idxSensor,2)/sqrt(curAccelRaw(idxSensor,1)^2 + curAccelRaw(idxSensor,3)^2));
        calib_A(idxSensor,idx,:) = [Ax Ay];
        %fprintf('sensor %0.0f, Ax=%+0.4f, Ay=%+0.4f\n', idxSensor, rad2deg(Ax), rad2deg(Ay));
    end
    %fprintf('\n');
    timeLapsed = toc;
end

launch_angle(numClusters,2) = zeros();
for idxCluster = 1:numClusters
    launch_angle(idxCluster,:) = [mean(calib_A(idxCluster,:,1)) mean(calib_A(idxCluster,:,2))];
end

%% Launch Detection
disp('Waiting for Launch Detection');
tic;
idx     =   1;
errors  =   0;
rawObs(1,12)         = zeros();
launch_accel_raw(numClusters,1,3)     = zeros();
launch_dif_accel_raw(numClusters,1,3)     = zeros();
isLaunchDetected = 0;
while not(isLaunchDetected)
    idx=idx+1;
    rawObs(idx,12)                = zeros();
    launch_accel_raw(:,idx,3)     = zeros();
    launch_accel_norm(:,idx,4)    = zeros();
    
    reading=fscanf(uart,'%s');          %Serial read
    
    if(length(reading)==36)
        for j=1:numSensors
            %Split arduino string of all readings to 3 char single readings
            %Convert 3 char raw reading string to double then to G's
            readingstr = char(reading((j-1)*3+1:(j-1)*3+3));
            rawObs(idx ,j) = (str2double(readingstr)-512)/102.3*9.81;
        end
    else
        continue;
    end
    
    oldAccelRaw     = curAccelRaw;         
    oldAccelMag     = curAccelMag;
    oldAccelUvector = curAccelUvector;
    curAccelRaw(1,:) = [  rawObs(idx,2)    rawObs(idx,1)  rawObs(idx,3) ]; % left wing
    curAccelRaw(2,:) = [  rawObs(idx,5)    rawObs(idx,4)  rawObs(idx,6) ]; % right wing
    curAccelRaw(3,:) = [  rawObs(idx,7)   -1*rawObs(idx,8) rawObs(idx,9) ]; % nose
    curAccelRaw(4,:) = [ -1*rawObs(idx,10)   rawObs(idx,11) rawObs(idx,12) ];     % back sensor
    for idxSensor = 1:numClusters
        [curAccelMag(idxSensor) curAccelUvector(idxSensor,:)] = NormalizeVector(curAccelRaw(idxSensor,:));
        difAccelRaw(idxSensor,:) = curAccelRaw(idxSensor,:) - oldAccelRaw(idxSensor,:);
        [difAccelMag(idxSensor) difAccelUvector(idxSensor,:)] = NormalizeVector(difAccelRaw(idxSensor,:));
    end

    sensorsOT = 0; % sensors overthreshold
    for idxSensor = 1:numClusters
        if (difAccelMag > LaunchThreshMinAccel)
            sensorsOT = sensorsOT + 1;
        end
    end 
    if (sensorsOT > LaunchThreshNumSensors)
        isLaunchDetected = 1;
    end
    launch_accel_raw(:,  idx , 1:3)  = curAccelRaw;
    launch_dif_accel_raw(:,  idx , 1:3)  = difAccelRaw;
end
disp('Launch Detected');

%% In flight monitor
disp('Switching to in-flight monitor');
tic;
idx     =   1;
inflight_time(idx) = toc;
errors  =   0;
rawObs(1,12)                  = zeros();
inflight_accel_raw(numClusters,1,3)     = zeros();
inflight_A(numClusters,1,2) = zeros();
isMaxHeight = 0;
maxHeightTime = 0;
while true
    idx=idx+1;
    inflight_time(idx)   = toc;
    
    rawObs(idx,12)         = zeros();        %May be unnecessary? already initialized?
    inflight_accel_raw(numClusters,idx,3)     = zeros();
    inflight_A(numClusters,idx,2)             = zeros();
    
    reading=fscanf(uart,'%s');          %Serial read
    if(length(reading)==36)
        for j=1:numSensors
            %Split arduino string of all readings to 3 char single readings
            %Convert 3 char raw reading string to double then to G's
            readingstr = char(reading((j-1)*3+1:(j-1)*3+3));
            rawObs(idx ,j) = (str2double(readingstr)-512)/102.3*9.81;
        end
    else
        continue;
    end

    curAccelRaw(1,:) = [  rawObs(idx,2)    rawObs(idx,1)  rawObs(idx,3) ]; % left wing
    curAccelRaw(2,:) = [  rawObs(idx,5)    rawObs(idx,4)  rawObs(idx,6) ]; % right wing
    curAccelRaw(3,:) = [  rawObs(idx,7)   -1*rawObs(idx,8) rawObs(idx,9) ]; % nose
    curAccelRaw(4,:) = [ -1*rawObs(idx,10)   rawObs(idx,11) rawObs(idx,12) ];     % back sensor
    
    for idxSensor = 1:numClusters
        [curAccelMag(idxSensor) curAccelUvector(idxSensor,:)] = NormalizeVector(curAccelRaw(idxSensor,:));
        Ax = atan(curAccelRaw(idxSensor,1)/sqrt(curAccelRaw(idxSensor,2)^2 + curAccelRaw(idxSensor,3)^2));
        Ay = atan(curAccelRaw(idxSensor,2)/sqrt(curAccelRaw(idxSensor,1)^2 + curAccelRaw(idxSensor,3)^2));
        inflight_A(idxSensor,idx,:) = [Ax Ay];
        %fprintf('sensor %0.0f, Ax=%+0.4f, Ay=%+0.4f\n', idxSensor, rad2deg(Ax), rad2deg(Ay));
        [difAccelMag(idxSensor) difAccelUvector(idxSensor,:)] = NormalizeVector(difAccelRaw(idxSensor,:));
    end
    
    if not(isMaxHeight)
        numSensorThresh = 0;
        for idxSensor = 1:numClusters
            Ax = inflight_A(idxSensor,idx,1);
            if (Ax < 0)
                numSensorThresh = numSensorThresh + 1;
            end
        end
        if (numSensorThresh >= 3)
            isMaxHeight = 1;
            maxHeightTime = inflight_time(idx);
        end
        %fprintf('\n');
    else
        fprintf('\tHeight Detected after %+0.4f\n',maxHeightTime);
    end
    inflight_accel_raw(:,  idx, 1:3)  = curAccelRaw;

    

    pause(0.0002);
end
