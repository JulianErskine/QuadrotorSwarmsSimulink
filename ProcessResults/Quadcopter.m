classdef Quadcopter < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    %% Properties
    properties (Constant)
        m = 0.8; % Quadcopter m
        rotorRadius = 0.1; % Rotor Radius
        g = [0,0,-9.81,0,0,0].'; % Gravity Acceleration
        maxThrust = 32;
        kf = 1.858E-5;
        km = 0.005022;
        cT = 0.00001;
    end
    properties (SetAccess = immutable)
        rotorTransform; % Geometric Pattern of Rotors
        rotorControlMatrix;
        invRotorControlMatrix;
    end
    properties (SetAccess = protected)
        rotationMatrix; % Quadcopter Rotation Matrix
        rotorThrust; % Rotor thrust Vector for the quadcopter
        thrusterWrench; % Actuation wrench on the quadcopter
        rotorTorques;
        rotorSpeeds;
        maxSupport = 10; % The maximum load the quadcopter can support along this external wrench vector
    end
    properties
        pose; % Quadcopter Pose
        twist;  % Quadcopter Twist
        twist2; % Quadcopter acceleration
        cableWrench;    % Force exterted on quadcopter by cable
        externalWrench; % External Wrench acting on the Quadcopter
        beta;
    end
    
    %% Methods
    methods 
        %% Construction
        %%%%%%%%%    Constructor %%%%%%%%%%
        function obj = Quadcopter()
            xspacing = 0.25;
            yspacing = 0.25;
            l = sqrt(xspacing^2+yspacing^2);
            zoffset = 0;
            obj.rotorTransform = [-xspacing/2,-yspacing/2,zoffset;...
                                    xspacing/2,-yspacing/2,zoffset;...
                                    xspacing/2, yspacing/2,zoffset;...
                                    -xspacing/2, yspacing/2,zoffset].';
            
            obj.pose = [1,1,1,0,0,0].';
            obj.calculateRotationMatrix;
            obj.externalWrench = obj.m*obj.g;
            obj.cableWrench = [0,0,0,0,0,0].';
            obj.beta = 0;
            obj.twist = [0,0,0,0,0,0].';
            obj.twist2 = [0,0,0,0,0,0].';
            obj.thrusterWrench = -(obj.cableWrench+obj.externalWrench)...
                                 +(obj.m*obj.twist2);
            kf = obj.kf;
            km = obj.km;
            M = [kf,    kf,    kf, kf;
                  0, -l*kf,     0, l*kf;
               l*kf,     0, -l*kf, 0;
                 km,   -km,    km, -km]; 
            obj.rotorControlMatrix = M;
            obj.invRotorControlMatrix = inv(M);
        end

        %% Actions
        
        function obj = balanceQuadcopter(obj)
            obj.calculateRotationMatrix;
            obj.calculateThrusterWrench;
            obj.computeOrientation;
            obj.calculateRotationMatrix;
            obj.calculateRotorThrusts;
            obj.calculateMaxTension;
            obj.calculateRotorSpeeds;
        end

        %% Setters
        
        %%%%%%%%%    Setters    %%%%%%%%%%%%%%
        % compute new rotation matrix when pose is updated
        function obj = set.pose(obj,value)
            [a,b] = size(value);
            if  a ~= 6 || b ~= 1
                error('Pose must be a 6x1 vector')
            end
            obj.pose = value;
        end
        
        % set position
        function obj = set_position(obj, value)
          obj.pose(1:3) = value;
        end 
        
        % set orientation
        function obj = set_orientation(obj, value)
          [a,b] = size(value);
          if  a == 1 && b == 4
            obj.rotationMatrix = quat2rotm(value);
          elseif  a == 4 && b == 1
            obj.rotationMatrix = quat2rotm(value');
          else
            error("the orientation set should be a quaternion")
          end
        end 
        %% Computations
        %%%%%%%%%%%% Computations %%%%%%%%%%%%%        
        
        %Calculate Rotation Matrix
        function obj = calculateRotationMatrix(obj)
            obj.rotationMatrix = eul2rotm(obj.pose(4:6).','ZYX');
        end
        
        % Calculate rotor Thrust
        function obj = calculateRotorThrusts(obj)
            % compute inverse dynamic model
            value = 3;
        end
        
        function obj = calculateThrusterWrench(obj)
            obj.thrusterWrench = -obj.cableWrench-obj.externalWrench...
                                 +(obj.m*obj.twist2);
        end
        
        function obj = computeOrientation(obj) % close form, not optimization
            Fz = obj.thrusterWrench(3);
            Ft = sqrt(obj.thrusterWrench(1)^2+obj.thrusterWrench(2)^2);
            phi = atan2(Fz,Ft);
            obj.pose(5) = pi/2-phi;
        end
        
        function obj = calculateMaxTension(obj)
            f = obj.maxThrust;
            m = obj.m;
            g = norm(obj.g);
            g = g + obj.twist2(3); % For verticle acceleration testing only!!!!!!!!
            B = obj.beta;
            obj.maxSupport = sqrt(f^2-m^2*g^2*cos(B)^2)-m*g*sin(B);
        end
        
        function obj = calculateRotorSpeeds(obj)
            R = obj.rotationMatrix;
            invR = inv(R);
            Fquad = invR*obj.thrusterWrench(1:3);
            Mquad = invR*obj.thrusterWrench(4:6);
            quadFrameActuationWrench = [Fquad(3);Mquad]; %[Fz, Mx, My, Mz] is quadcopter frame
            omegaSquared = obj.invRotorControlMatrix * quadFrameActuationWrench;
            obj.rotorSpeeds = sqrt(omegaSquared);
            obj.rotorTorques = obj.km*omegaSquared;
        end
        
        %% Plotting
        
        %%%%%%%%%%%%%%% Plot Function %%%%%%%%%%%%%%%%
        function plotHandle = plotQuadcopter(obj)
            pose = obj.pose(1:3);
            r = obj.rotorRadius;
            Rot = obj.rotationMatrix;
            
            % generate rotor matrices
            rotorCenters = obj.rotorTransform;
            resolution = 12;
            x_circle = zeros(resolution,4);
            y_circle = zeros(resolution,4);
            z_circle = zeros(resolution,4);
            % for each rotor
            for i = 1:4
                % approximate circle
                for j = 1:resolution
                    circle(1,j) = rotorCenters(1,i)+r*cos(2*pi*(j-1)/(resolution-1));
                    circle(2,j) = rotorCenters(2,i)+r*sin(2*pi*(j-1)/(resolution-1));
                    circle(3,j) = rotorCenters(3,i);
                end
                RotorPlots(:,:,i) = Rot*circle + ones(3,resolution).*pose;
            end
            
            % Generate crossbeam matrices
            beamEnds = pose.*ones(3,4)+Rot*obj.rotorTransform;
            crossBeam1 = [beamEnds(:,1),beamEnds(:,3)];
            crossBeam2 = [beamEnds(:,2),beamEnds(:,4)];

            % Generate Frames
            Xaxis = [pose,pose + 3*r*Rot*[1;0;0]];
            Yaxis = [pose,pose + 3*r*Rot*[0;1;0]];
            Zaxis = [pose,pose + 3*r*Rot*[0;0;1]];
            
            % Plot
            h(1) = plot3(Xaxis(1,:),Xaxis(2,:),Xaxis(3,:),'r','linewidth',3);
            h(2) = plot3(Yaxis(1,:),Yaxis(2,:),Yaxis(3,:),'g','linewidth',3);
            h(3) = plot3(Zaxis(1,:),Zaxis(2,:),Zaxis(3,:),'b','linewidth',3);
            h(4) = plot3(pose(1),pose(2),pose(3),'k*','linewidth',5,'MarkerSize',5);
            % plot beams
            h(5) = plot3(crossBeam1(1,:),crossBeam1(2,:),crossBeam1(3,:),'k','linewidth',2);
            h(6) = plot3(crossBeam2(1,:),crossBeam2(2,:),crossBeam2(3,:),'k','linewidth',2);

            for i = 1:4
                h(6+i) = plot3(RotorPlots(1,:,i),RotorPlots(2,:,i),RotorPlots(3,:,i),'k','linewidth',2);
            end
            
            plotHandle = [h];
        end % end quadcopter plot
        
    end % end methods
end % end Quadcopter class

