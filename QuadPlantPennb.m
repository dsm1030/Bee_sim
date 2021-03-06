classdef QuadPlantPennb < SecondOrderSystem
    % Modified from D. Mellinger, N. Michael, and V. Kumar, 
    % "Trajectory generation and control for precise aggressive maneuvers with quadrotors",
    %  In Proceedings of the 12th International Symposium on Experimental Robotics (ISER 2010), 2010. 
    
    methods
        function obj = QuadPlantPennb()
            obj = obj@SecondOrderSystem(6,4,1);
            obj = setStateFrame(obj,CoordinateFrame('QuadState',12,'x',{'x','y','z','roll','pitch','yaw','xdot','ydot','zdot','rolldot','pitchdot','yawdot'}));
            obj = obj.setOutputFrame(obj.getStateFrame);
        end
        
        function m = getMass(obj)
          m = obj.m
        end
        
        function I = getInertia(obj)
          I = obj.I;
        end
        
        function u0 = nominalThrust(obj)
          % each propellor commands -mg/4
          u0 = Point(getInputFrame(obj),obj.m*9.81*[0;0;0;1]);
        end
        
        function qdd = sodynamics(obj,t,q,qd,u)
            % States
            % x
            % y
            % z
            % phi (roll)
            % theta (pitch)
            % psi (yaw)
            % xdot
            % ydot
            % zdot
            % phidot
            % thetadot
            % psidot
            
            % Parameters
            m = obj.m;
            I = obj.I;
            thrust_vector = obj.thrust_vector;
            invI = diag(1./[0.0023,0.0023,0.004]);
            g = 9.81;
            L = 0.1750;
            
            % states
            x = [q;qd];
                        
            phi = x(4);
            theta = x(5);
            psi = x(6);

            phidot = x(10);
            thetadot = x(11);
            psidot = x(12);
            
            tau_pitch = u(1);
            tau_yaw = u(2);
            tau_roll = u(3);
            thrust = u(4);
            
            % Rotation matrix from body to world frames
            R = rpy2rotmat([phi;theta;psi]);
            
            xyz_ddot = (1/m)*([0;0;-m*g] + R*thrust_vector*thrust);
            
            pqr = rpydot2angularvel([phi;theta;psi],[phidot;thetadot;psidot]);
            pqr = R'*pqr;

            pqr_dot = invI*([tau_roll;tau_pitch;tau_yaw] - cross(pqr,I*pqr));
  
            % Now, convert pqr_dot to rpy_ddot
            [Phi, dPhi] = angularvel2rpydotMatrix([phi;theta;psi]);
            
            Rdot =  [ 0, sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta),   cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta); ...
                      0, cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta); ...
                      0,                              cos(phi)*cos(theta),                               -cos(theta)*sin(phi)]*phidot + ...
                      [ -cos(psi)*sin(theta), cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(psi)*cos(theta); ...
                        -sin(psi)*sin(theta), cos(theta)*sin(phi)*sin(psi), cos(phi)*cos(theta)*sin(psi); ...
                                -cos(theta),         -sin(phi)*sin(theta),         -cos(phi)*sin(theta)]*thetadot + ...
                               [ -cos(theta)*sin(psi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta), cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta); ...
                                 cos(psi)*cos(theta),   cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta); ...
                                                   0,                                                  0,                                                0]*psidot;
  
            rpy_ddot = Phi*R*pqr_dot + reshape((dPhi*[phidot;thetadot;psidot]),3,3)*R*pqr + ...
                       Phi*Rdot*pqr;

            % xdot = [x(7:12);xyz_ddot;rpy_ddot];
            qdd = [xyz_ddot;rpy_ddot];
            
            
        end
        
        function y = output(obj,t,x,u)
            y = x;
        end
        
        function x = getInitialState(obj)
            x = zeros(12,1);
        end
        
        
        function v = constructVisualizer(obj)
          r = Quadrotorb();
          v = r.constructVisualizer();
          tf = AffineTransform(obj.getOutputFrame(),v.getInputFrame(),[eye(6),zeros(6)],zeros(6,1));
          obj.getOutputFrame().addTransform(tf);
        end
    end
    properties
      m = .5;
      I = diag(.0023*[.1, 1, 1]); %roll, pitch, yaw
      thrust_vector = [cos(60*pi/180); 0; sin(60*pi/180)];
      
    end
    
end



