classdef Drone < handle
    
%% MEMBERS    
    properties
        g
        t
        dt
        tf
        
        m
        l
        I
        
        x                                                                  %(X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r)
        r                                                                  %(X, Y, Z)
        dr                                                                 %(dX, dY, dZ)
        euler                                                              %(phi, theta, psi)
        w                                                                  %(p, q, r)
        
        dx
        
        u
        T
        M
    end
    
    properties
        phi_des
        phi_err
        phi_err_prev
        phi_err_sum
        
        theta_des
        theta_err
        theta_err_prev
        theta_err_sum
        
        psi_des
        psi_err
        psi_err_prev
        psi_err_sum
        
        zdot_des
        zdot_err
        zdot_err_prev
        zdot_err_sum
    end
    
    properties
        KP_phi
        KI_phi
        KD_phi
        
        KP_theta
        KI_theta
        KD_theta
        
        KP_psi
        KI_psi
        KD_psi
        
        KP_zdot
        KI_zdot
        KD_zdot
    end
    
    
%% METHODS
    methods
    %% CONSTRUCTOR
        function obj = Drone(params, initStates, initInputs, gains, simTime)
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = 0.01;
            obj.tf = simTime;
            
            obj.m = params('mass');
            obj.l = params('armLength');
            obj.I = [params('Ixx'),0,0 ; 0,params('Iyy'),0; 0,0,params('Izz')];
            
            obj.x = initStates;
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);
            
            obj.dx = zeros(12,1);
            
            obj.u = initInputs;
            obj.T = obj.u(1);
            obj.M = obj.u(2:4);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.phi_err = 0.0;
            obj.phi_err_prev = 0.0;
            obj.phi_err_sum = 0.0;
            obj.theta_err = 0.0;
            obj.theta_err_prev = 0.0;
            obj.theta_err_sum = 0.0;
            obj.psi_err = 0.0;
            obj.psi_err_prev = 0.0;
            obj.psi_err_sum = 0.0;
            
            obj.zdot_err = 0.0;
            obj.zdot_err_prev = 0.0;
            obj.zdot_err_sum = 0.0;
			
            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Find proper gains for the controller.
            obj.KP_phi=gains('P_phi');
            obj.KI_phi=gains('I_phi');
            obj.KD_phi=gains('D_phi');
            
            obj.KP_theta=gains('P_theta');
            obj.KI_theta=gains('I_theta');
            obj.KD_theta=gains('D_theta');
            
            obj.KP_psi=gains('P_psi');
            obj.KI_psi=gains('I_psi');
            obj.KD_psi=gains('D_psi');
            
            obj.KP_zdot = gains('P_zdot');
            obj.KI_zdot = gains('I_zdot');
            obj.KD_zdot = gains('D_zdot');
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
    %% RETURNS DRONE STATE
        function state = GetState(obj)
            state = obj.x;
        end
        
    %% STATE SPACE (DIFFERENTIAL) EQUATIONS: INCOMPLETE!
        function obj = EvalEOM(obj)
            bRi = RPY2Rot(obj.euler);            
            R = bRi';                                                    
            
            % Translational Motions
            obj.dx(1:3) = obj.dr;
            obj.dx(4:6) = 1 / obj.m * ([0; 0; obj.m * obj.g] + R * obj.T * [0; 0; -1]);
            
            % Rotational Motions
            phi = obj.euler(1); theta = obj.euler(2);
            obj.dx(7:9) = [1    sin(phi)*tan(theta) cos(phi)*tan(theta);
                           0    cos(phi)            -sin(phi);
                           0    sin(phi)*sec(theta) cos(phi)*sec(theta)] * obj.w;
                       
            obj.dx(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w));

        end

    %% PREDICT NEXT DRONE STATE
        function obj = UpdateState(obj)
            obj.t = obj.t + obj.dt;
            
            % Find(update) the next state of obj.X
            obj.EvalEOM();
            obj.x = obj.x + obj.dx.*obj.dt;
            
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Simulate the actual sensor output you can measure from the drone.
            % Examples down here are not accurate model 
%             obj.w(1) = obj.w(1) + randn() + 0.2;
%             obj.w(2) = obj.w(2) + randn() + 0.2;
%             obj.w(3) = obj.w(3) + randn() + 0.5;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
        end
        
    %% CONTROLLER
        function obj = AttitudeCtrl(obj, refSig)
			obj.phi_des = refSig(1);
			obj.theta_des = refSig(2);
			obj.psi_des = refSig(3);
			obj.zdot_des = refSig(4);
			
            obj.phi_err = obj.phi_des - obj.euler(1);
            obj.theta_err = obj.theta_des - obj.euler(2);
            obj.psi_err = obj.psi_des - obj.euler(3);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Write a code for the rate controller (PID controller for now)
            % that produces RPY moments. Position of the drone may change
            % since we are dealing with the attitude of the drone only.
            % (e.g.) obj.M(1,1) = ~~~; obj.M(2,1) = ~~~; obj.M(3,1) = ~~~; 
            obj.u(2) = (obj.KP_phi * obj.phi_err + ...
                        obj.KI_phi * (obj.phi_err_sum) + ...
						obj.KD_phi * (0 - obj.w(1))); % With small angle Approx.
						%(Diff. is not stable in micro_processors such as DSP, ARM)(Gain might differ)
%                         obj.KD_phi * (obj.phi_err - obj.phi_err_prev)/obj.dt);

						
                      
            obj.u(3) = (obj.KP_theta * obj.theta_err + ...
                        obj.KI_theta * (obj.theta_err_sum) + ...
                        obj.KD_theta * (obj.theta_err - obj.theta_err_prev)/obj.dt);
                      
            obj.u(4) = (obj.KP_psi * obj.psi_err + ...
                        obj.KI_psi * (obj.psi_err_sum) + ...
                        obj.KD_psi * (obj.psi_err - obj.psi_err_prev)/obj.dt);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
            
            obj.phi_err_prev = obj.phi_err;
            obj.phi_err_sum = obj.phi_err_sum + obj.phi_err;
            obj.theta_err_prev = obj.theta_err;
            obj.theta_err_sum = obj.theta_err_sum + obj.theta_err;
            obj.psi_err_prev = obj.psi_err;
            obj.psi_err_sum = obj.psi_err_sum + obj.psi_err;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Basic motor output assumed to make our problem easier.
            % Remove it when you want to put a realistic simulation
            % environment with the actual motor dynamics.
            obj.zdot_err = obj.zdot_des - obj.dr(3);
            
            obj.u(1) = obj.m * obj.g;
            obj.u(1) = obj.m * obj.g - ...
                       (obj.KP_zdot * obj.zdot_err + ...
                        obj.KI_zdot * (obj.zdot_err_sum) + ...
                        obj.KD_zdot * (obj.zdot_err - obj.zdot_err_prev)/obj.dt);
                    
            obj.zdot_err_prev = obj.zdot_err;
            obj.zdot_err_sum = obj.zdot_err_sum + obj.zdot_err;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             obj.u(1) = 2*obj.m * obj.g;
%             obj.u(2) = 0.0;
%             obj.u(3) = 0.0;
%             obj.u(4) = 0.0;

            obj.T = obj.u(1);
            obj.M = obj.u(2:4);
        end
    end
end

