classdef classBalancingRobotFlow
    properties
        mc;
        Jc;
        mw;
        Jw;
        l;
        r;
        g;
        b_phi;
        b_theta;
    end
    
    methods
        % Computes and returns the value of theta dot
        function thetaDot = dtheta(obj, q, ptheta, omega, alpha)
            % Extract states
            theta = q(2);
            
            % Compute intermediate variables from Task 1.1/2.1 derivations
            A = obj.Jw + obj.mw*obj.r^2 + obj.mc*obj.r^2;
            B = obj.mc*obj.r*obj.l*cos(alpha + theta);
            C = obj.Jc + obj.mc*obj.l^2;
            
            % Compute A_new and B_new parameters
            A_new = 1 ./ (A + 2*B + C);
            B_new = (A + B) ./ (A + 2*B + C);
            
            % Evaluate thetaDot
            thetaDot = A_new.*ptheta - B_new.*omega;
        end
        
        % Computes and returns the value of p_theta dot
        function pDot = dptheta(obj, q, ptheta, omega, alpha)
            % Extract states
            theta = q(2);
            
            % Re-use the dtheta method to get current thetaDot
            thetaDot = obj.dtheta(q, ptheta, omega, alpha);
            
            % Evaluate momentum dynamics using the derived equation
            term1 = -obj.mc * obj.r * obj.l * sin(alpha + theta) * (omega*thetaDot + thetaDot^2);
            term2 = -(obj.mw + obj.mc) * obj.g * obj.r * sin(alpha);
            term3 = obj.mc * obj.g * obj.l * sin(theta);
            term4 = -obj.b_theta * thetaDot;
            
            pDot = term1 + term2 + term3 + term4;
        end

        % Computes and returns the time derivative of the state vector
        function dx = state_derivative(obj, x, omega, alpha)
            % Extract states from x vector
            phi = x(1);
            theta = x(2);
            ptheta = x(3);
            
            % Assemble q vector required for helper methods
            q = [phi; theta];
            
            % Compute derivatives
            dphi = omega;
            thetaDot = obj.dtheta(q, ptheta, omega, alpha);
            pDot = obj.dptheta(q, ptheta, omega, alpha);
            
            % Assemble and return dx
            dx = [dphi; thetaDot; pDot];
        end
    end
end