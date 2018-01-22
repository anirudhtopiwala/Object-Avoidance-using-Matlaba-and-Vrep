vrep=remApi('remoteApi');
 vrep.simxFinish(-1); 
 clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
 dpcenter = 1;i=0;
 if (clientID>-1)
        disp('Connected');

%         code here
while( dpcenter~='null' )
i=i+1;
%         Handle
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking );
    [returnCode,Right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking );
    [returnCode,front_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking );
    [returnCode,left_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor3',vrep.simx_opmode_blocking );
    [returnCode,right_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7',vrep.simx_opmode_blocking );

    [returnCode,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking );

%     Other Code

% Reading sensor readings
    [returnCode,detectionStatecenter,dpcenter,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionStateleft,dpleft,~,~]=vrep.simxReadProximitySensor(clientID,left_Sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionStateright,dpright,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor,vrep.simx_opmode_streaming);

    dpcenter= norm(dpcenter);
    dpleft= norm(dpleft);
    dpright= norm(dpright);
     
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,3,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,3,vrep.simx_opmode_blocking); 
   

    if ( dpcenter<0.35 && dpcenter>0.001 || dpleft<0.35 && dpleft>0.001 || dpright<0.35 && dpright>0.001)
        if ( dpleft<0.35 && dpleft>0.001 && dpright< 0.01 && dpcenter< 0.01)
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0.2,vrep.simx_opmode_blocking); 
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,-0.2,vrep.simx_opmode_blocking);
             pause(0.05)
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking); 

        elseif (dpright<0.35 && dpright>0.001 && dpleft< 0.01 && dpcenter< 0.01)
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,-0.2,vrep.simx_opmode_blocking); 
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0.2,vrep.simx_opmode_blocking);
             pause(0.05)
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking);
%         else
%             a= power(-1,i);
%             if(a==1)
%                 [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,-0.2,vrep.simx_opmode_blocking); 
%                 [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0.4,vrep.simx_opmode_blocking);
%                  pause(2.5)
%                 [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);
%                 [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking);
%             else
%             [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,-0.2,vrep.simx_opmode_blocking); 
%             [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0.4,vrep.simx_opmode_blocking);
%              pause(2.5)
%             [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);
%             [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking);
            end
        
                [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,-0.5,vrep.simx_opmode_blocking); 
                [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0.5,vrep.simx_opmode_blocking);
                 pause(0.2)
                [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking);

    end
 
    
    
    [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_streaming);
    [returnCode,leftlinearvelocity, leftangularvelocity]=vrep.simxGetObjectVelocity(clientID,left_Motor,vrep.simx_opmode_streaming);
    [returnCode,rightlinearvelocity, rightangularvelocity]=vrep.simxGetObjectVelocity(clientID,Right_Motor,vrep.simx_opmode_streaming);

    [returnCode,leftlinearvelocity,leftangularvelocity]=vrep.simxGetObjectVelocity(clientID,left_Motor,vrep.simx_opmode_buffer);
    [returnCode,rightlinearvelocity, rightangularvelocity]=vrep.simxGetObjectVelocity(clientID,Right_Motor,vrep.simx_opmode_buffer);
    [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
    imshow(image)
    disp(norm(dpcenter));
    disp(norm(dpleft));
    disp(norm(dpright));
%     disp(leftlinearvelocity);
%     disp(rightlinearvelocity);
%     disp(leftangularvelocity);
%     disp(rightangularvelocity);

     
     
   end
  
    vrep.simxFinish(-1); 
 end
 
 vrep.delete();
 