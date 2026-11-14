classdef classDiscreteLQRcontrol < handle

    properties
        controlMode % Indicator of active control mode.
        T           % Sampling period of controller.
        A           % Linearised A matrix for controller.
        B           % Linearised B matrix for controller.
        C_r         % Linearised C matrix for regulation output.
        D_r         % Linearised D matrix for regulation output.
        A_int       % A matrix augmented with integrator.
        B_int       % B matrix augmented with integrator.
        Q           % Q matrix for LQR cost.
        R           % R matrix for LQR cost.
        N           % N matrix for LQR cost.
        K           % Optimal control gain.
        N_x         % Feed-forward gain for states.
        N_u         % Feed-forward gain for inputs.
        y_ref       % Output regulation reference.
        A_z         % State coefficient for integrator transition dynamics.
        B_z         % Input coefficient for integrator transition dynamics.
        z           % Integrator state.
    end

    methods

        function controlInit_regulation(obj, A,B,Q,R,N,T) % computes optimal lqr state-fb gain
            obj.A = A;
            obj.B = B;
            obj.Q = Q;
            obj.R = R;
            obj.N = N;
            obj.T = T;

            obj.controlMode = "Regulation";

            obj.K = lqrd(obj.A,obj.B,obj.Q,obj.R,obj.N,obj.T);

        end


        function u = control(obj,x) % computes the control input based on provided states
            %not accurate btw --->  %input x is the error_x between our state and where we want to be

            %switch between control modes
            switch obj.controlMode
                %regulation only control
                case 'Regulation'
                    u = -obj.K*x;
                case 'Integrator'
                    % Compute feedback from the augmented state.
                    state = [x; obj.z];
                    u = -obj.K*state;
                    % Update integrator state for next iteration
                    obj.z = obj.A_z*state + obj.B_z*u; % this second bit comes out to 0
                case 'Reference'
                    u_star = obj.N_u * obj.y_ref;
                    u = u_star - (obj.K*x); %lec 08 01 slides 12 and 13
                case 'Reference_integrator'
                    x_star = obj.N_x*obj.y_ref;
                    error_x = x - x_star; % also x_squiggly in lec08
                    state = [error_x; obj.z];
                    u_star = obj.N_u * obj.y_ref;
                    u_bar = -obj.K*state; % lec08 01 slide 16
                    
                    u = u_star + u_bar;
                    
                    obj.z = obj.A_z*state + obj.B_z*u_bar; %idek, but realistically in the assignment when it asks you to make A_z and B_z
                    %obj.z =  obj.z + obj.C_r*(x - x_star); % lec08 01 slide 17
                    

                otherwise
                    u = 0;
            end
        end

        function controlInit_integrator(obj,A,B,C_r,D_r,Q,R,N,T)
            obj.A = A;
            obj.B = B;
            obj.C_r = C_r;
            obj.D_r = D_r;
            obj.Q = Q;
            obj.R = R;
            obj.N = N;
            obj.T = T;

            obj.A_int = [obj.A zeros(4,1); -obj.C_r 0];
            obj.B_int = [obj.B; -obj.D_r];


            % Define the descrete-time integrator update matricies using an Euler
            % discretisation. ie. Matrices that satisfy:
            % z_(k+1) = A_z*[x_k-x_star; z_k] + B_z*u_k

            % bro wtf I do not understand this
            obj.A_z = [-obj.T * obj.C_r, 1];
            obj.B_z = -obj.T * obj.D_r;

            obj.K = lqrd(obj.A_int, obj.B_int, obj.Q, obj.R, obj.N, obj.T);

            obj.z = 0;

            obj.controlMode = 'Integrator';
        end

        function controlInit_reference(obj, A,B,C_r,D_r,Q,R,N,T,y_ref) 
            %computes optimal LQR state feedback gain and feedb fwd gains for given reference
            obj.A = A;
            obj.B = B;
            obj.C_r = C_r;
            obj.D_r = D_r;
            obj.Q = Q;
            obj.R = R;
            obj.N = N;
            obj.T = T;
            obj.y_ref = y_ref;

            % Construct N vector and extract feed-forward gains Nx, Nu
            partial_N_ff = inv([obj.A obj.B; obj.C_r obj.D_r]);
            partial_N_ff_2 = [zeros(size(obj.A, 1), size(obj.C_r,1)) ; eye(size(obj.C_r,1))];
            N_feedforward = partial_N_ff * partial_N_ff_2; %lec08 01
            obj.N_x = N_feedforward(1:size(obj.A,2), :); %gets our x states. We know how many there are because A must have as many cols as x states
            obj.N_u = N_feedforward(size(obj.A,2)+1 :end, :); %gets u states which are the rest (or the num rows of B)

            %compute optimal control gain
            obj.K = lqrd(obj.A,obj.B,obj.Q,obj.R,obj.N,obj.T);

            obj.controlMode = "Reference";
        end

        function controlInit_reference_integrator(obj,A,B,C_r,D_r,Q,R,N,T,y_ref)
            % Store properties
            obj.A = A;
            obj.B = B;
            obj.C_r = C_r;
            obj.D_r = D_r;
            obj.Q = Q;
            obj.R = R;
            obj.N = N;
            obj.T = T;
            obj.y_ref = y_ref;

            % Construct Nff vector and extract feed-forward gains Nx, Nu
            partial_N_ff = inv([obj.A obj.B; obj.C_r obj.D_r]);
            partial_N_ff_2 = [zeros(size(obj.A, 1), size(obj.C_r,1)) ; eye(size(obj.C_r,1))];
            N_feedforward = partial_N_ff * partial_N_ff_2; %lec08 01
            obj.N_x = N_feedforward(1:size(obj.A,2), :); %gets our x states. We know how many there are because A must have as many cols as x states
            obj.N_u = N_feedforward(size(obj.A,2)+1 :end, :); %gets u states which are the rest (or the num rows of B)


            % Construct continuous-time augmented A B matrices with integrator
            obj.A_int = [obj.A zeros(3,1); obj.C_r 0]; %lec08 01 slide 16
            obj.B_int = [obj.B; obj.D_r]; %lec08 01 slide 16

            %Define the discrete-time integrator update matricies using an Euler
            % discretisation. ie. Matrices that satisfy:
            %z_(k+1)=A_z*[x_k-x_star;z_k]+B_z*u_k

            obj.A_z = [obj.T * obj.C_r, 1]; %lec08 01 slide 16 (it is just the bottom row)
            obj.B_z = obj.T * obj.D_r; %lec08 01 slide 16 (it is just the bottom row)

            %Compute optimal control gain
            obj.K= lqrd(obj.A_int, obj.B_int, obj.Q, obj.R, obj.N, obj.T);

            %Initialize integrator
            obj.z = 0;

            %Update indicator
            obj.controlMode='Reference_integrator';
        end

    end


end