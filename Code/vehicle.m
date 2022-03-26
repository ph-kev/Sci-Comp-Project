classdef Vehicle<handle
    %VEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        width
        state
        params
        lane
    end
    
    methods
        function obj = Vehicle(x_0, v_0, a_0, width)
            %VEHICLE Construct an instance of this class
            %   Detailed explanation goes here
            obj.width = width;
            obj.state = [x_0, v_0, a_0];
            % optional parameters
            %varargin_param_names = {'adaptation time', 'v_desired', 'transition_width', 'form_factor', 'time_gap', 'min_gap'};
            obj.params = [.65, 33.3, 15, 1.5, 1.4, 3];
            %if (nargin > length(varargin_default_values))
            %    error('Too many input parameters');
            %end
            %obj.params = varargin_default_values;
            %obj.params(1:nargin) = varargin;
            obj.lane = 1;
        end
        
        function v_opt = v_opt(obj, s)  % s is the gap between the cars 
            v_0 = obj.params(2);
            T = obj.params(5);
            s_0 = obj.params(6);
            v_opt = max(0,min(v_0,(s-s_0)/(T)));
        end
        
        function newstate = timestep(obj, s, v_l, t_step, gamma)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            tau = obj.params(1); 
            oldstate = obj.state;
            x_a = oldstate(1);
            v_a = oldstate(2);
            acc = ((obj.v_opt(s) - v_a)/tau) - gamma*(v_a - v_l);
            vel = v_a + acc * t_step; 
            pos = x_a + ((v_a + vel)/2)*t_step; 
            newstate = [pos, vel, acc];
            obj.state = [pos, vel, acc];
        end
    end
end

