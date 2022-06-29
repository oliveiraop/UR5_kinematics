clear all
close all
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
        disp('Connected');
        vrep.simxAddStatusbarMessage(clientID,'Matlab Connected',vrep.simx_opmode_oneshot); 
        %% Handle 
    [~, Joint1]   = vrep.simxGetObjectHandle(clientID,'UR5_joint1',vrep.simx_opmode_blocking);
    [~, Joint2]   = vrep.simxGetObjectHandle(clientID,'UR5_joint2',vrep.simx_opmode_blocking);
    [~, Joint3]   = vrep.simxGetObjectHandle(clientID,'UR5_joint3',vrep.simx_opmode_blocking);
    [~, Joint4]   = vrep.simxGetObjectHandle(clientID,'UR5_joint4',vrep.simx_opmode_blocking);
    [~, Joint5]   = vrep.simxGetObjectHandle(clientID,'UR5_joint5',vrep.simx_opmode_blocking);
    [~, Joint6]   = vrep.simxGetObjectHandle(clientID,'UR5_joint6',vrep.simx_opmode_blocking);
    [~, Gripper]   = vrep.simxGetObjectHandle(clientID,'ROBOTIQ_85',vrep.simx_opmode_blocking);
    [~, Cuboid]  = vrep.simxGetObjectHandle(clientID,'Cuboid',vrep.simx_opmode_blocking);
    [~, Copo]  = vrep.simxGetObjectHandle(clientID,'Cup',vrep.simx_opmode_blocking);
    [returnCode,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
    [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_streaming);
     

% Enable the synchronous mode on the client:
vrep.simxSynchronous(clientID,true);

% Start the simulation:
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);

% Pose Home
    vrep.simxSetJointTargetPosition(clientID,Joint1,degtorad(0),vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,Joint2,degtorad(0),vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,Joint3,degtorad(0),vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,Joint4,degtorad(0),vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,Joint5,degtorad(0),vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,Joint6,degtorad(0),vrep.simx_opmode_oneshot)
    
 % Anda um passo de simulacao
    vrep.simxSynchronousTrigger(clientID);

% Periodo de amostragem e tempo de simulacao
T = 0.05; % Tempo de passo. Tem de estar em concordancia com Vrep!
Tsim = 2;  % segundos
t = 0:T:Tsim;
N = size(t,2);
        
      
% Loop da simulacao
for i=1:N-1
 
    % Exemplo de movimentacao das juntas 
    vrep.simxSetJointTargetPosition(clientID,Joint1,degtorad(90),vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,Joint2,degtorad(0),vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,Joint3,degtorad(90),vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,Joint4,degtorad(0),vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,Joint5,degtorad(-90),vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,Joint6,degtorad(0),vrep.simx_opmode_oneshot)
    [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_buffer);
           
    imshow(image)  
    
    % Anda um passo de simulacao
    vrep.simxSynchronousTrigger(clientID);
    
       
end


        
vrep.simxAddStatusbarMessage(clientID,'Matlab DisConnected',vrep.simx_opmode_oneshot); 
%pause(.1);      
        
vrep.simxSynchronousTrigger(clientID);
         
% stop the simulation:
vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

% Now close the connection to CoppeliaSim:    
vrep.simxFinish(clientID);
        

end

 
vrep.delete(); % call the destructor!
disp('Program ended');
 
