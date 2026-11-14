classdef classBalancingRobotFlowLinear
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
        % computes and returns the A matrix of the linearized balancing robot system
        function A_lin = A(obj)
            A_lin = [0, 0, 0;
                0, 0, 1/(obj.Jc + obj.Jw + obj.l^2*obj.mc + obj.mc*obj.r^2 + obj.mw*obj.r^2 + 2*obj.l*obj.mc*obj.r);
                0, obj.mc*obj.g*obj.l, -obj.b_theta/(obj.Jc + obj.Jw + obj.l^2*obj.mc + obj.mc*obj.r^2 + obj.mw*obj.r^2 + 2*obj.l*obj.mc*obj.r)];
        end

        % computes and returns the B matrix of the linearized balancing robot system.
        function B_lin = B(obj)
                B_lin = [1;
                -(obj.Jw + obj.mc*obj.r^2 + obj.mw*obj.r^2 + obj.l*obj.mc*obj.r)/(obj.Jc + obj.Jw + obj.l^2*obj.mc + obj.mc*obj.r^2 + obj.mw*obj.r^2 + 2*obj.l*obj.mc*obj.r);
                (obj.b_theta*(obj.Jw + obj.mc*obj.r^2 + obj.mw*obj.r^2 + obj.l*obj.mc*obj.r))/(obj.Jc + obj.Jw + obj.l^2*obj.mc + obj.mc*obj.r^2 + obj.mw*obj.r^2 + 2*obj.l*obj.mc*obj.r)];
        end

        % computes and returns the time derivative of the state vector 
        % according to the linearised model.
        function dx = state_derivative(obj, x, omega)
            
            dx = obj.A() * x + obj.B() * omega;

        end
    end
end