classdef Bot_youBot < handle
    properties (Access = private)
        CoppeliaSim = [];
        ClientID = [];
        
        Joints = [];

        ReferenceFrame = [];
    end
    
    methods
        function obj = Bot_youBot()
            try
                obj.CoppeliaSim = remApi('remoteApi');
                obj.CoppeliaSim.simxFinish(-1);
                obj.ClientID = obj.CoppeliaSim.simxStart('127.0.0.1',19997,true,true,5000,5); 
                obj.CoppeliaSim.simxStopSimulation(obj.ClientID,obj.CoppeliaSim.simx_opmode_oneshot_wait);

                obj.CoppeliaSim.simxSynchronous(obj.ClientID,true);
                obj.CoppeliaSim.simxStartSimulation(obj.ClientID,obj.CoppeliaSim.simx_opmode_blocking);

                [~,obj.Joints(1)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotArmJoint0',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(2)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotArmJoint1',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(3)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotArmJoint2',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(4)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotArmJoint3',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(5)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotArmJoint4',obj.CoppeliaSim.simx_opmode_oneshot_wait);

                [~,obj.ReferenceFrame] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/ReferenceFrame',obj.CoppeliaSim.simx_opmode_oneshot_wait);
            catch E
                error(E)
            end
        end
        
        function q = Get_Joint_Position (obj)
            try
                aux = ones(5,1);
                q = zeros(5,1);
                
                while sum(aux)~=0
                    [aux(1),q(1)] = obj.CoppeliaSim.simxGetJointPosition(obj.ClientID,obj.Joints(1),obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(2),q(2)] = obj.CoppeliaSim.simxGetJointPosition(obj.ClientID,obj.Joints(2),obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(3),q(3)] = obj.CoppeliaSim.simxGetJointPosition(obj.ClientID,obj.Joints(3),obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(4),q(4)] = obj.CoppeliaSim.simxGetJointPosition(obj.ClientID,obj.Joints(4),obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(5),q(5)] = obj.CoppeliaSim.simxGetJointPosition(obj.ClientID,obj.Joints(5),obj.CoppeliaSim.simx_opmode_streaming);
                end
            catch E
                error(E)
            end
        end

        function [T] = ReferenceFrame_Pose (obj)
            try
                paux = 1; oaux = 1;
                
                while paux==1 || oaux==1
                    [paux,p] = obj.CoppeliaSim.simxGetObjectPosition(obj.ClientID,obj.ReferenceFrame,-1,obj.CoppeliaSim.simx_opmode_streaming);
                    [oaux,o] = obj.CoppeliaSim.simxGetObjectOrientation(obj.ClientID,obj.ReferenceFrame,-1,obj.CoppeliaSim.simx_opmode_streaming);

                    p = double(p)'; 
                    
                    o = rad2deg(double(o))';
                    R = rotx(o(1))*roty(o(2))*rotz(o(3));
                    
                    T = [[R p]; 0 0 0 1];
                end
            catch E
                error(E)
            end
        end

        function obj = Set_Joint_Velocity (obj,qp)
            try
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(1),qp(1),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(2),qp(2),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(3),qp(3),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(4),qp(4),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(5),qp(5),obj.CoppeliaSim.simx_opmode_streaming);
            catch E
                error(E)
            end
        end

        function Simulation_Step (obj)
            try
                obj.CoppeliaSim.simxSynchronousTrigger(obj.ClientID);  
            catch E
                error(E)
            end
        end

        function Stop_Simulation (obj)
            try
                obj.CoppeliaSim.simxStopSimulation(obj.ClientID,obj.CoppeliaSim.simx_opmode_oneshot_wait);
            catch E
                error(E)
            end
        end
    end
end
