classdef Drone < handle
    
%% MEMBERS    
    properties
        g
        t
        dt
        END_TIME
        
        mass
        L
        I
        
        X                                                                       %(x, y, z, dx, dy, dz, q1, q2, q3, q4, p, q, r)
        r                                                                       %(x, y, z)
        dr                                                                      %(dx, dy, dz)
        quat                                                                    %(q1, q2, q3, q4)
        w                                                                       %(p, q, r)
        
        phi
        psi
        theta
        
        dX
        
        input
        F
        M
    end
    
    properties
        p_des
        p_err
        p_err_prev
        p_err_sum
        
        q_des
        q_err
        q_err_prev
        q_err_sum
        
        r_des
        r_err
        r_err_prev
        r_err_sum
    end
    
    properties
        KP_p
        KI_p
        KD_p
        KP_q
        KI_q
        KD_q
        KP_r
        KI_r
        KD_r
    end
    
    
%% METHODS
    methods
    %% CONSTRUCTOR: COMPLETED
        function obj = Drone(initStates, initInputs, params, tf)
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = 0.01;
            obj.END_TIME = tf;
            
            obj.mass = params('mass');
            obj.L = params('armLength');
            obj.I = [params('Ixx'),0,0 ; 0,params('Iyy'),0; 0,0,params('Izz')];
            
            obj.X = initStates;
            obj.r = obj.X(1:3);
            obj.dr = obj.X(4:6);
            obj.quat = obj.X(7:10);
            [obj.phi, obj.psi, obj.theta] = Rot2RPY_ZXY(Quat2RM(obj.quat));
            obj.w = obj.X(11:13);
            
            obj.dX = zeros(13,1);
            
            obj.input = initInputs;
            obj.F = obj.input(1);
            obj.M = obj.input(2:4);
            
            obj.p_err = 0.0;
            obj.p_err_prev = 0.0;
            obj.p_err_sum = 0.0;
            obj.q_err = 0.0;
            obj.q_err_prev = 0.0;
            obj.q_err_sum = 0.0;
            obj.r_err = 0.0;
            obj.r_err_prev = 0.0;
            obj.r_err_sum = 0.0;
            
            obj.p_des = 0.0;
            obj.q_des = 0.0;
            obj.r_des = 0.0;

            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Find proper gains for the controller.
            obj.KP_p=0.008;
            obj.KI_p=0.004;
            obj.KD_p=0.001;
            
            obj.KP_q=obj.KP_p;
            obj.KI_q=obj.KI_p;
            obj.KD_q=obj.KD_p;
            
            obj.KP_r=0.01;
            obj.KI_r=0.006;
            obj.KD_r=0.0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        end
    %% RETURNS DRONE STATE: COMPLETED 
        function state = GetState(obj)
            state = obj.X;
        end
        
    %% STATE SPACE (DIFFERENTIAL) EQUATIONS: INCOMPLETE!
        function obj = EvalEOM(obj)
            % World to Body Rotation Matrix
            bRw = Quat2RM(obj.quat);            
            
            % Body to World Rotation Matrix
            wRb = bRw';                                                    
            
            % Find dx/dt, dy/dt, dz/dt (velocity, obj.dX(1:3))
            obj.dX(1:3) = obj.dr;
            
            % Find d^2x/dt^2, ... (acceleration, obj.dX(4:6))
            obj.dX(4:6) = 1 / obj.mass * (wRb * [0; 0; obj.F] - [0; 0; obj.mass * obj.g]);
            
            % Angular velocity
            Kq = 2;
            qerr = 1 - (obj.quat(1)^2 + obj.quat(2)^2 + obj.quat(3)^2 + obj.quat(4)^2);
            obj.dX(7:10) = -1/2*[       0, -obj.w(1), -obj.w(2), -obj.w(3);...
                                 obj.w(1),         0, -obj.w(3),  obj.w(2);...
                                 obj.w(2),  obj.w(3),         0, -obj.w(1);...
                                 obj.w(3), -obj.w(2),  obj.w(1),         0] * obj.quat + Kq*qerr *obj.quat;

            % Angular acceleration
            obj.dX(11:13) = (obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w));

        end

    %% PREDICT NEXT DRONE STATE
        function obj = UpdateState(obj)
            obj.t = obj.t + obj.dt;
            
            % Find(update) the next state of obj.X
            obj.EvalEOM();
            obj.X = obj.X + obj.dX.*obj.dt;
            
            obj.r = obj.X(1:3);
            obj.dr = obj.X(4:6);
            obj.quat = obj.X(7:10);
            [obj.phi, obj.psi, obj.theta] = Rot2RPY_ZXY(Quat2RM(obj.quat));
            
            obj.w = obj.X(11:13);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Simulate the actual sensor output you can measure from the drone.
            % Examples down here are not accurate model 
            obj.w(1) = obj.w(1) + randn() + 0.2;
            obj.w(2) = obj.w(2) + randn() + 0.2;
            obj.w(3) = obj.w(3) + randn() + 0.5;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
        end
        
    %% CONTROLLER
        function obj = UpdateController(obj)
            obj.p_err = obj.p_des - obj.w(1);
            obj.q_err = obj.q_des - obj.w(2);
            obj.r_err = obj.r_des - obj.w(3);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Write a code for the rate controller (PID controller for now)
            % that produces RPY moments. Position of the drone may change
            % since we are dealing with the attitude of the drone only.
            % (e.g.) obj.M(1,1) = ~~~; obj.M(2,1) = ~~~; obj.M(3,1) = ~~~; 
            obj.M(1,1) = (obj.KP_p * obj.p_err + ...
                          obj.KI_p * (obj.p_err_sum) + ...
                          obj.KD_p * (obj.p_err - obj.p_err_prev));
                      
            obj.M(2,1) = (obj.KP_q * obj.q_err + ...
                          obj.KI_q * (obj.q_err_sum) + ...
                          obj.KD_q * (obj.q_err - obj.q_err_prev));
                      
            obj.M(3,1) = (obj.KP_r * obj.r_err + ...
                          obj.KI_r * (obj.r_err_sum) + ...
                          obj.KD_r * (obj.r_err - obj.r_err_prev));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
            
            obj.p_err_prev = obj.p_err;
            obj.p_err_sum = obj.p_err_sum + obj.p_err;
            obj.q_err_prev = obj.q_err;
            obj.q_err_sum = obj.q_err_sum + obj.q_err;
            obj.r_err_prev = obj.r_err;
            obj.r_err_sum = obj.r_err_sum + obj.r_err;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Basic motor output assumed to make our problem easier.
            % Remove it when you want to put a realistic simulation
            % environment with the actual motor dynamics.
            obj.F = obj.mass * obj.g;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end

