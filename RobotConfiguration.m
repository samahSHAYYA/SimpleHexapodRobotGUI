classdef RobotConfiguration < handle & dynamicprops
% ROBOTCONFIGURATION Contains all data that describes the robot
%                    configuration. It includes actuated joints' positions 
%                    and pose. This class inherits from both handle (to act
%                    as reference class) and dynamicprops (to allow adding
%                    custom properties depending on robot type). This class
%                    can be used with serial and parallel robots.
% 
% Example:
%        rc = RobotConfiguration([], Pose);
%        rc.addprop("someAdditionalProp"); % adding someAdditionalProp
%        rc.someAdditionalProp = 58; % setting value to someAdditionalProp

    properties
        % Actuated joints' positions.
        q (:, 1) double

        % Robot's pose (end-effector pose for instance).
        pose (1, 1) Pose

        % Flags to determine if is reachable (i.e. within joints' limits)
        % and collision-free.
        isReachable (1, 1) logical
        isCollisionFree (1, 1) logical
    end

    % These contain dependent (computable) properties.
    properties (Dependent)
        isValid (1, 1) logical
    end
    
    methods
        function obj = RobotConfiguration(q, pose)
        % ROBOTCONFIGURATION Constructs an instance of this class.
        % 
        % Input:
        %  - q: actuated joints' positions vector.
        %  - pose: robot's pose.

            arguments (Input)
                q (:, 1) double
                pose (1, 1) Pose
            end
            
            obj.q = q;
            obj.pose = pose;
        end

        % Get method for isValid property.
        function tf = get.isValid(obj)
        % ISVALID Tells if the current configuration is valid, meaning
        %         both reachable and collision-free.
          
            tf = obj.isReachable && obj.isCollisionFree;
        end

        function clonedObj = clone(obj)
        % CLONE Creates a copy of this current object to assure
        %       independency as obj is handle object not a value one.
        
            clonedObj = RobotConfiguration(obj.q, obj.pose);
            clonedObj.isReachable = obj.isReachable;
            clonedObj.isCollisionFree = obj.isCollisionFree;

            % Now, we need to copy dynamic properties.
            fixedProps = properties(class(obj));
            allProps = properties(obj); % These include dynamic ones.

            % Dynamic properties is set difference.
            dynProps = setdiff(allProps, fixedProps);

            for i = 1 : numel(dynProps)
                clonedObj.addprop(dynProps{i});
                clonedObj.(dynProps{i}) = obj.(dynProps{i});
            end
        end
    end
end
