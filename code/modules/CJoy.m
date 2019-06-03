classdef CJoy < handle
    %CJoy Handles all joystick commands
    %   Detailed explanation goes here
    
    properties
        %% Indices mapping to buttons on joystick
        X_VEL_AXIS      = 4;
        Y_VEL_AXIS      = 3;
        Z_VEL_AXIS      = 2;
        YAW_SPEED_AXIS  = 1;
        START_BUTTON    = 2;
        LAND_BUTTON     = 3;
        AUTO_BUTTON     = 1;
        MANUAL_BUTTON   = 4;
        START_BUTTOM    = 10;
                
        %% Commands
        vel = [0;0;0];
        yawSpeed = 0;
        
        %% Joystick modes
        land = 0
        start = 0
        manual = 1
        auto = 0
        mpc = 0
        
        %% ROS subscriber/publisher
        Joy_sub
        
    end
    
    methods
        
        function obj = CJoy()
            % Constructor
            % ROS subscriber initialise
            obj.Joy_sub = rossubscriber('/bebop/joy','sensor_msgs/Joy',@obj.Joy_callback);
        end
        
         %%  =======================================================================
         
        function Joy_callback(obj,data,data1)
            % Callback (triggered when Joy_sub gets new message
            % Get velocity and yawspeed data
            obj.vel = [data1.Axes(obj.X_VEL_AXIS);
                data1.Axes(obj.Y_VEL_AXIS);
                data1.Axes(obj.Z_VEL_AXIS)];
                        
            obj.yawSpeed = data1.Axes(obj.YAW_SPEED_AXIS);
                        
            % Set modes
            if data1.Buttons(obj.START_BUTTON) == 1
                obj.start  = 1;
                obj.land   = 0;
                obj.auto   = 0;
                obj.manual = 1;
                obj.mpc    = 0;
            end
            
            if data1.Buttons(obj.LAND_BUTTON) == 1
                obj.start = 0;
                obj.land = 1;
                obj.auto = 0;
                obj.manual = 1;
                obj.mpc = 0;
            end
          
            if data1.Buttons(obj.AUTO_BUTTON) == 1
                obj.auto = 1;
                obj.manual = 0;
                obj.mpc = 0;
            end
            
            if data1.Buttons(obj.MANUAL_BUTTON) == 1
                obj.auto = 0;
                obj.manual = 1;
                obj.mpc = 0;
            end
            
             if data1.Buttons(obj.START_BUTTOM) == 1
                obj.auto = 0;
                obj.manual = 0;
                obj.mpc = 1;
             end
            
        end
        
        function [vel,yawSpeed] = getCommands(obj)
            % Returns velocity and yawSpeed commands
            vel = obj.vel;
            yawSpeed = obj.yawSpeed;
        end
        
        function flagNewMPC = setDroneState(obj,Drone)
            % Set CDrone object's mode according to Joy mode
            % If the mpc mode is just activated then set flagNewMPC to 1
            % else it is 0
            flagNewMPC = 0;
            
            if obj.start == 1 && Drone.m_flying == 0               
                Drone.takeoff();
                Drone.m_flying  = 1;
                Drone.m_auto    = 0;
                Drone.m_manual  = 1;
                Drone.m_mpc     = 0;
                Drone.m_ppi     = 0;
            end 
            
            if obj.land == 1
                Drone.land();
                Drone.m_flying  = 0;
                Drone.m_auto    = 0;
                Drone.m_manual  = 1;
                Drone.m_mpc     = 0;
                Drone.m_ppi     = 0;
            end                      
            
            if obj.auto == 1 && Drone.m_flying == 1
                Drone.m_ppi     = 1;
                Drone.m_auto    = 0;
                Drone.m_manual  = 0;
                Drone.m_mpc     = 0;
            end
            
            if obj.manual == 1
                Drone.m_ppi     = 0;
                Drone.m_auto    = 0;
                Drone.m_manual  = 1;
                Drone.m_mpc     = 0;
            end
            
            if obj.mpc == 1
                % If the drone is currently not in MPC, then trigger this 
                % as first time setting MPC mode
                if Drone.m_mpc == 0
                    flagNewMPC = 1;
                end
                
                Drone.m_auto = 0;
                Drone.m_manual = 0;
                Drone.m_mpc = 1;
            end
         
        end
               
    end
    
end

