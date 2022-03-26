classdef vehicle<handle
    %VEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        length
        state
        params
    end
    
    methods
        function obj = vehicle(x_0, v_0, a_0, length, varargin)
            %VEHICLE Construct an instance of this class
            %   Detailed explanation goes here
            obj.length = length;
            obj.state = [x_0, v_0, a_0];
            % optional parameters
            varargin_param_names = {'adaptation time', 'v_desired', 'transition_width', 'form_factor', 'time_gap', 'min_gap'};
            varargin_default_values = {0.65, 33.3, 15, 1.5, 1.4, 3};
            if (length(varargin)>length(varargin_param_names))
                error('Too many input parameters');
            end
            obj.params = varargin_default_values;
            obj.params(1:nargin) = varargin;
        end
        
        function v_opt = v_opt(obj, s)  % s is the gap between the cars 
            [~, v_0, ~, ~, T, s_0] = obj.params; 
            v_opt = max(0,min(v_0,(s-s_0)/(T)));
        end
        
        function newstate = timestep(obj, s, v_l, t_step, gamma)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            tau = obj.params(1); 
            [x_a, v_a, ~] = obj.state;
            acc = ((v_opt(s) - v_a)/tau) - gamma*(v_a - v_l);
            vel = v_a + acc * t_step; 
            pos = x_a + ((v_a + vel)/2)*t_step; 
            newstate = [pos, vel, acc];
            obj.state = newstate;
        end
    end
end

