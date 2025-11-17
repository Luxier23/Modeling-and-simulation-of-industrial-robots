classdef Bot_Scara < handle
    properties (Access = private)
        CoppeliaSim = [];
        ClientID = [];
        
        Joints = [];
        Gripper = [];
        
        Rectangle = [];
        ReferenceFrame = [];

        q = [];
    end
    
    methods
        function obj = Bot_Scara()
            obj.CoppeliaSim = remApi('remoteApi');
            obj.CoppeliaSim.simxFinish(-1);
            obj.ClientID = obj.CoppeliaSim.simxStart('127.0.0.1',19997,true,true,5000,5); 
            disp('Connecting to robot....')
            
            obj.CoppeliaSim.simxStopSimulation(obj.ClientID,obj.CoppeliaSim.simx_opmode_oneshot_wait);
            obj.CoppeliaSim.simxStartSimulation(obj.ClientID,obj.CoppeliaSim.simx_opmode_blocking);

            if obj.ClientID==0
                [~,obj.Joints(1)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/MTB/axis1',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(2)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/MTB/axis2',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(3)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/MTB/axis3',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(4)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/MTB/axis4',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Gripper] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/BaxterGripper/closeJoint',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                
                [~,obj.Rectangle] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/Rectangle',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.ReferenceFrame] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/ReferenceFrame',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                pause(1)

                obj.q = [0 0 0 0]';
                obj.CoppeliaSim.simxSetJointPosition(obj.ClientID,obj.Joints(1),obj.q(1),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointPosition(obj.ClientID,obj.Joints(2),obj.q(2),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointPosition(obj.ClientID,obj.Joints(3),obj.q(3),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointPosition(obj.ClientID,obj.Joints(4),-obj.q(4),obj.CoppeliaSim.simx_opmode_streaming);
                
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Gripper,-0.05,obj.CoppeliaSim.simx_opmode_streaming);
                disp(' done.')
            else
                error(' robot not connected...')
            end
        end
        
        function obj = Gripper_Command (obj,action) % action = 0 para cerrar, action = 1 para abrir
            if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                if action == 0
                    obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Gripper,0.05,obj.CoppeliaSim.simx_opmode_streaming);
                elseif action == 1
                    obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Gripper,-0.05,obj.CoppeliaSim.simx_opmode_streaming);
                end
            else  
                error(' connection lost...')
            end
        end
        
        function obj = Set_Joint_Position (obj,q)
            if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                obj.CoppeliaSim.simxSetJointPosition(obj.ClientID,obj.Joints(1),q(1),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointPosition(obj.ClientID,obj.Joints(2),q(2),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointPosition(obj.ClientID,obj.Joints(3),q(3),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointPosition(obj.ClientID,obj.Joints(4),-q(4),obj.CoppeliaSim.simx_opmode_streaming);
                obj.q = q;
            else
                error(' connection lost...')
            end
        end

        function q = Get_Joint_Position (obj)
            q = obj.q;
        end
        
        function T = Rectangle_Pose (obj)
            paux = 1; oaux = 1;
            
            while paux==1 || oaux==1
                if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                    [paux,p] = obj.CoppeliaSim.simxGetObjectPosition(obj.ClientID,obj.Rectangle,-1,obj.CoppeliaSim.simx_opmode_streaming);
                    [oaux,o] = obj.CoppeliaSim.simxGetObjectOrientation(obj.ClientID,obj.Rectangle,-1,obj.CoppeliaSim.simx_opmode_streaming);

                    p = double(p'); o = double(o');

                    Rx = [1 0 0; 0 cos(o(1)) -sin(o(1));  0 sin(o(1)) cos(o(1))];
                    Ry = [cos(o(2)) 0 sin(o(2)); 0 1 0;  -sin(o(2)) 0 cos(o(2))];
                    Rz = [cos(o(3)) -sin(o(3)) 0; sin(o(3)) cos(o(3)) 0;  0 0 1];

                    T = [[Rx*Ry*Rz p]; 0 0 0 1];
                else
                    error(' connection lost...')
                end
            end
        end
        
        function T = ReferenceFrame_Pose (obj)
            paux = 1; oaux = 1;
            
            while paux==1 || oaux==1
                if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                    [paux,p] = obj.CoppeliaSim.simxGetObjectPosition(obj.ClientID,obj.ReferenceFrame,-1,obj.CoppeliaSim.simx_opmode_streaming);
                    [oaux,o] = obj.CoppeliaSim.simxGetObjectOrientation(obj.ClientID,obj.ReferenceFrame,-1,obj.CoppeliaSim.simx_opmode_streaming);

                    p = double(p'); o = double(o');

                    Rx = [1 0 0; 0 cos(o(1)) -sin(o(1));  0 sin(o(1)) cos(o(1))];
                    Ry = [cos(o(2)) 0 sin(o(2)); 0 1 0;  -sin(o(2)) 0 cos(o(2))];
                    Rz = [cos(o(3)) -sin(o(3)) 0; sin(o(3)) cos(o(3)) 0;  0 0 1];

                    T = [[Rx*Ry*Rz p]; 0 0 0 1];
                else
                    error(' connection lost...')
                end
            end
        end

        function s = Connection(obj)
            s = obj.CoppeliaSim.simxGetConnectionId(obj.ClientID);
        end

        function Stop_Simulation (obj)
            obj.CoppeliaSim.simxStopSimulation(obj.ClientID,obj.CoppeliaSim.simx_opmode_oneshot_wait);
        end
    end
end
