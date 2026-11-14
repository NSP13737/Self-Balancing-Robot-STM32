classdef classDCMotor
    properties
        Ra %armature resistance
        La %armature inductance
        N %gear ratio
        Td %friction torque in the motor
        Ko %gyrator gain K_omega
        Ki %gyrator gain K_i
        Vmax %maximum motor voltage
        Vmin %min motor voltage
        Jm % effective inertia of everything between armature and output (gearbox + wheels).
           % Jm is estimated from free wheel spin test
        b % motor friction? (for our model we will just take average of friction in both directions
        Kp % PI controller gain
    end

    methods
        function dlambda = state_derivative(obj,V_in,lambda,omega_m)
            %return the derivative of magnetic flux
            dlambda = V_in - obj.Ko*obj.N*omega_m - (obj.Ra/obj.La)*lambda;
        end
        
        function I_a = armature_current(obj,lambda)
            I_a = lambda/obj.La;
        end
        
        function tau_a = dead_band(obj,I_a)
            if (obj.Ki*I_a) < -obj.Td
                tau_a = obj.Ki*I_a + obj.Td;
            elseif (obj.Ki*I_a) > obj.Td
                tau_a = obj.Ki*I_a - obj.Td;
            else
                tau_a = 0;
            end
        end

        function t_m = output_torque(obj,lambda)

            I_a = obj.armature_current(lambda);
            tau_a = obj.dead_band(I_a);
            t_m = obj.N * tau_a;

        end

        function w_m = output_vel(obj,V_in,lambda)

            w_m = (V_in - obj.Ra*obj.armature_current(lambda)) / (obj.Ko*obj.N);

        end

        function V_in = control_allocation(obj, tau_m, omega_m)
            %computes the required motor voltage V_in within 
            % the actuator limits for a desired tau_m and the 
            % current wheel angular velocity
            if tau_m < 0
                I_a = ((1/obj.N)*tau_m - obj.Td) / obj.Ki;
            elseif tau_m > 0
                I_a = ((1/obj.N)*tau_m + obj.Td) / obj.Ki;
            else
                I_a = 0;
            end

            V_in = obj.Ra*I_a + obj.Ko*obj.N*omega_m;
            if (V_in > obj.Vmax)
                V_in = obj.Vmax;
            elseif (V_in < obj.Vmin)
                V_in = obj.Vmin;
            end
            

        end

        function Tm = get_motor_time_constant(obj)
            Tm = (obj.Ra*obj.Jm) / (obj.N^2*obj.Ko*obj.Ki + obj.b*obj.Ra)
        end

        function Vin = flowControlAllocation(obj, desired_vel, current_vel, sample_period)

            error_vel = desired_vel - current_vel;

            persistent z %integrator error
            if isempty(z)
                z = 0;
            end

            %% Update z (integrator) state based on Roxy's equations
            z = z + ( (obj.Kp*sample_period/obj.get_motor_time_constant()) * error_vel);

            %%
            Vin = obj.Kp*error_vel + z;
            if (Vin > obj.Vmax)
                Vin = obj.Vmax;
            elseif (Vin < obj.Vmin)
                Vin = obj.Vmin;
            end

        end


    end

end