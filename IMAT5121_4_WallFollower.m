% Program: Script to make Pioneer 3-DX bot follow a wall in V-REP
% Name: IMAT5121_4_WallFollower.m
% Author: Wim Nagler
% Date: 12/11/2020
% simRemoteApi.start(19999)

% Load V-Rep Library, close existing connection and create new sim
vrep=remApi('remoteApi');       
vrep.simxFinish(-1);            
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% Program Control Parameters 
randomOrStraight = 0;   % 0= straight forward, 1= random walk a, 2= random walk b
wallFollowAlgo = 1;     % 0=Controller A basic algoritm, 1=Controller B PD Controller       
s=200;                  % Ping frequency of the sonars in Hertz

% Parameters random walk
maxStraightTime = 1;    % Max time the robot can run straight
maxWheelSpeed = 5;      % Max speed the robot can turn
maxTurnTime=1.5;        % Max length of time the robot can turn

% Parameters Wall following algorithm
safeFrontDistance = 1; 
minFrontDistance = 0.3;
targetRightDistance=0.45;   %Desired right distance to the wall
errorRightDistance=0.25 ;    %Tolerance right distance to the wall
speed=3 ;                   %Forward speed of the P3DX
turn=2.1;                   % Maximum turn velocity
gain=20;                    %PD controller gain setting


if (clientID>-1)
    % If connection successful
    disp('Connected')
    % Create handles for required V-Rep objects
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    [returnCode,front_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
    [returnCode,right_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8',vrep.simx_opmode_blocking);
        
    % Initialise sensors
    [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState,distanceRight,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor,vrep.simx_opmode_streaming);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Find Wall %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if randomOrStraight == 1
        % Random Walk a = separate turn and straight driving
        while norm(distanceFront) >= safeFrontDistance | norm(distanceFront)<1e-10
            % Make a random turn
            turnR=-2*maxWheelSpeed*rand+maxWheelSpeed  %Setting random wheel motion between -maxWheelSpeed and +maxWheelSpeed
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, turnR ,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, -turnR ,vrep.simx_opmode_blocking);
            pause(-maxTurnTime*rand+maxTurnTime)
            
            % Drive straight for a random time 
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, maxWheelSpeed ,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, maxWheelSpeed ,vrep.simx_opmode_blocking);
            pause(-maxStraightTime*rand+maxStraightTime)
            [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
        end
        
    elseif randomOrStraight == 2    
        % Random Walk b = joint turning and driving
            % Keep one wheel running at constant speed equal to 1/2 maxWheelSpeed
            % and modify the other randomly between 0 and maxWheelSpeed
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, maxWheelSpeed/2 ,vrep.simx_opmode_blocking);
        while norm(distanceFront) >= safeFrontDistance | norm(distanceFront)<1e-10
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, -maxWheelSpeed*rand+maxWheelSpeed ,vrep.simx_opmode_blocking);
            pause(maxTurnTime)
            [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
        end        
        
    elseif randomOrStraight == 0
        %Start Driving straight
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, 3 ,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, 3 ,vrep.simx_opmode_blocking);
        
        % While wall not found keep driving straight
        while norm(distanceFront) >= safeFrontDistance | norm(distanceFront)<1e-10  %Or is needed to correct for very small values when nothing is detected. API Error?
            pause(1/s);
            [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
        end
    end
    
    % Stop because we are in front of the wall
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, 0 ,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, 0 ,vrep.simx_opmode_blocking);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END Find Wall %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Follow Wall  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if wallFollowAlgo == 0  %Controller A : Basic Algo
        for i=1:2000
            
            pause(1/s);
            
            % Ping sonars
            [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
            [returnCode,detectionState,distanceRight,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor,vrep.simx_opmode_buffer);
        
            % Setting the Front Flag
            if (norm(distanceFront) >= minFrontDistance) && (norm(distanceFront) <= safeFrontDistance)  %Safe
                frontFlag = 1;
            elseif (norm(distanceFront) <= minFrontDistance) && (norm(distanceFront) >= 1e-10)          %Close
                frontFlag = 2;
            else
                frontFlag = 3;  % Nothing in Front
            end
        
            % Setting Right Flag
            if (norm(distanceRight) >= targetRightDistance-errorRightDistance) && (norm(distanceRight) <= targetRightDistance+errorRightDistance)
                rightFlag = 1;      % Target zone
            elseif (norm(distanceRight) <= targetRightDistance-errorRightDistance) && (norm(distanceRight) >= 1e-10)
                rightFlag = 2;      % Too close
            else
                rightFlag = 3;      % Too far
            end
    
            if frontFlag == 1       %frontFlag = Normdistance => turn to the left
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, speed-turn ,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed+turn ,vrep.simx_opmode_blocking);
        
            elseif frontFlag == 2   %If the bot is too close, back out until the safe distance
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, -speed ,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, -speed ,vrep.simx_opmode_blocking);
                while (norm(distanceFront) <= safeFrontDistance) && (norm(distanceFront) >= 1e-10)
                    pause(0.1);
                    [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
                end    

            elseif rightFlag == 2   %rightFlag = 2 => turn a to the left
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, speed-turn ,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed+turn ,vrep.simx_opmode_blocking);
                  
            elseif rightFlag == 3   %rightFlag = 3 => turn to the right
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, speed+turn ,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed-turn ,vrep.simx_opmode_blocking);
     
            else                    %rightFlag = 1 => drive straight
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, speed ,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed ,vrep.simx_opmode_blocking);
 
            end
        end
        
    elseif wallFollowAlgo == 1  %Controller B - PD Controller
        prevError=0;            %to be able to calculate the direction of the error correction
        error=0;                %Error between the wall and the P3DX
        
        for i=1:2000 
    
             pause(1/s);
            
            % Ping sonars
            [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
            [returnCode,detectionState,distanceRight,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor,vrep.simx_opmode_buffer);
    
            % Setting the Front Flag
            if (norm(distanceFront) >= minFrontDistance) && (norm(distanceFront) <= safeFrontDistance)  %Safe
                frontFlag = 1;
            elseif (norm(distanceFront) <= minFrontDistance) && (norm(distanceFront) >= 1e-10)          %Close
                frontFlag = 2;
            else
                frontFlag = 3;  % Nothing in Front
            end
            
            % Setting Right Flag
            if (norm(distanceRight) >= targetRightDistance-errorRightDistance) && (norm(distanceRight) <= targetRightDistance+errorRightDistance)
                rightFlag = 1;  % Within the tolerance
            elseif (norm(distanceRight) <= targetRightDistance-errorRightDistance) && (norm(distanceRight) >= 1e-10)
                rightFlag = 2;  % Too Close
            else
                rightFlag = 3;  % Too Far
            end
    
            prevError = error;
            error = norm(distanceRight) - targetRightDistance;
            
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, speed ,vrep.simx_opmode_blocking);          
            
            if frontFlag == 1       %frontFlag = Normdistance => turn a lot to the left
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed+2*turn ,vrep.simx_opmode_blocking);
            
            elseif frontFlag == 2   %If the bot is too close, back out until the safe distance
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, -speed ,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, -speed ,vrep.simx_opmode_blocking);
                while (norm(distanceFront) <= safeFrontDistance) && (norm(distanceFront) >= 1e-10)
                    pause(0.1);
                    [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
                end    

            elseif rightFlag == 2   %rightFlag = 2 => turn a to the left
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed+2*turn ,vrep.simx_opmode_blocking);
                  
            elseif rightFlag == 3   %rightFlag = 3 => turn to the right
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed-2*turn ,vrep.simx_opmode_blocking);
     
            else                    %rightFlag = 1 => drive straight
                if prevError > error && error > 0       % Too Far & getting closer
                    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed ,vrep.simx_opmode_blocking);
                elseif prevError < error && error > 0   % Too Far & going further
                    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed-gain*error ,vrep.simx_opmode_blocking);
                elseif prevError > error && error < 0   % Too Close & getting further
                    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed-gain*error ,vrep.simx_opmode_blocking);
                elseif prevError < error && error < 0   % Too Close & getting closer
                    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, speed ,vrep.simx_opmode_blocking);
                end    
            end      
         
        end
    end
    
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, 0 ,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, 0 ,vrep.simx_opmode_blocking);
    
    % Stop the simulation
    vrep.simxFinish(-1);
 
else
    % Connection Failed
    disp('Failed connecting to remote API server')
end

[returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, 0 ,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, 0 ,vrep.simx_opmode_blocking);

% Call the destructor
vrep.delete();
