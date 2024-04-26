classdef Pose
% POSE Describes the TCP (Target Control Point) position and end-effector
%      orientation in 3D space. The orientation is based on ZYX Eulerian 
%      angles.

    properties
        % Cartesian coordinates.
        x (1, 1) double = 0
        y (1, 1) double = 0
        z (1, 1) double = 0

        % Euler Angles ZYX (Intrinsic Angles) or
        % Roll-Pitch-Yaw XYZ (Extrinsic Angles) meaning: R = Rz * Ry * Rx
        % In radians. Note here in validation pi cannot be used as literal
        % values are required.
        tz (1, 1) double {mustBeInRange(tz, -3.1416, 3.1416)} = 0
        ty (1, 1) double {mustBeInRange(ty, -1.5708, 1.5708)} = 0
        tx (1, 1) double {mustBeInRange(tx, -3.1416, 3.1416)} = 0
    end
    
    methods
        function obj = Pose(varargin)
        % POSE Constructs an instance of this class.
        %
        % Example:
        %   - p = Pose(x, y, z, tz, ty, tx)
        %   - p = Pose(y = 1, 'tz', pi/4)

            parser = inputParser;

            parser.CaseSensitive = true;

            % Getting classname dynamicaly and using it.
            parser.FunctionName = class(obj);


            % Define arguments (here all are optional, if required
            % you can use addrequired instead).
            addParameter(parser, "x", 0, @isnumeric);
            addParameter(parser, "y", 0, @isnumeric);
            addParameter(parser, "z", 0, @isnumeric);

            addParameter(parser, "tz", 0, @(x)mustBeInRange(x, -180, 180));
            addParameter(parser, "ty", 0, @(x)mustBeInRange(x, -90, 90));
            addParameter(parser, "tx", 0, @(x)mustBeInRange(x, -180, 180));

            parse(parser, varargin{:});

            obj.x = parser.Results.x;
            obj.y = parser.Results.y;
            obj.z = parser.Results.z;

            obj.tz = parser.Results.tz;
            obj.ty = parser.Results.ty;
            obj.tx = parser.Results.tx;
        end

        function cartesianPosition = p(obj)
        % P Returns the TCP (Target Control Point) Cartesian position
        %   vector [x; y; z].

            cartesianPosition = [obj.x; obj.y; obj.z];
        end

        function eulerOrientation = phi(obj)
        % PHI Returns Euler ZYX angles vector [tz; ty; tx].

            eulerOrientation = [obj.x; obj.y; obj.z];
        end

        function Rz = rotz(obj)
        % ROTZ Returns the rotation matrix about Z-axis.
        %         
            % Note that rotz accepts angle value in degrees.
            Rz = rotz(obj.tz * 180 / pi);
        end

        function Ry = roty(obj)
        % ROTY Returns the rotation matrix about Y-axis.
        
            % Note that roty accepts angle value in degrees.
            Ry = roty(obj.ty * 180 / pi);
        end

        function Rx = rotx(obj)
        % ROTX Returns the rotation matrix about x-axis.
        
            % Note that rotx accepts angle value in degrees.
            Rx = rotx(obj.tx * 180 / pi);
        end

        function R = rot(obj)
        % ROT Returns the resultant rotation matrix.

            R = obj.rotz * obj.roty * obj.rotx;
        end

        function q = getQuaternion(obj)
        % GETQUATERNION Returns the quaternion representation of the 
        %               resultant rotation.

            % Note that the sequence "ZYX" can be omitted but explictly
            % mentioned for clarity (despite being the default). Note the
            % Euler angles are expected as row vector.
            q = eul2quat(obj.phi', "ZYX");

            % Another alternate way, would be to use rotation matrix as
            % a start.
            % q_alt = rotm2quat(obj.rot);
        end

        function T = Teul2ang(obj)
        % TEUL2ANG Returns the matrix that maps Euler angles rates to 
        %          angular velocity.

            % Unit vectors along ex, ey and ez.
            ex = [1; 0; 0];
            ey = [0; 1; 0];
            ez = [0; 0; 1];

            T = [ez, obj.rotz * ey, obj.rotz * obj.roty * ex];
        end
    end
end
