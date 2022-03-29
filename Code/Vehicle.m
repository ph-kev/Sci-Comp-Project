classdef Vehicle<handle
    % VEHICLE Summary of this class goes here

    properties
        width % Length of car.
        state % Position, velocity, and acceleration.
        params % Adaptation time, v_desired, transition_width, form_factor, time_gap, min_gap.
        lane % Lane number.
    end

    methods
        function obj = Vehicle(x_0, v_0, a_0, width, varargin)
            % VEHICLE Construct an instance of this class
            % \brief Find the optimal velocity of the car.
            % \param obj, car.
            % \param s, gap of the car and the leading car.
            % \returns Vehicle's optimal velocity.
            if nargin > 0 % Check the number of parameters.
                obj.width = width;
                obj.state = [x_0, v_0, a_0];
                % Optional paramters.
                obj.params = [0.65, 33.3, 15, 1.5, 1.4, 3];
                if (length(varargin) > length(obj.params))
                    error('Too many input parameters');
                end
                varargin = cell2mat(varargin);
                for i=1:length(varargin)
                    obj.params(i) = varargin(i);
                end
                obj.lane = 1;
            end
        end

        function v_opt = v_opt(obj, s)  % s is the gap between the cars
            % \brief Find the optimal velocity of the car.
            % \param obj, car.
            % \param s, gap of the car and the leading car.
            % \returns Vehicle's optimal velocity.
            v_0 = obj.params(2);
            T = obj.params(5);
            s_0 = obj.params(6);
            v_opt = max(0,min(v_0,(s-s_0)/(T)));
        end

        function newstate = timestep(obj, s, v_l, t_step, gamma)
            % \brief Update the car's state using information of the car in
            % front of it.
            % \param obj, car which is getting updated.
            % \param s, gap of the car and the leading car.
            % \param v_l, velocity of the leading car. 
            % \param t_step, time step.
            % \param gamma, speed difference sensitivity. 
            % \returns Vehicle's state updated after a single time step. 
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
    methods(Static)
        function sortedCars = sortCars(carsArr)
            % \brief Sort the cars in descending order according to their
            % position.
            % \param carsArr, an array of car
            % \returns Sorted array of cars in descending order of their positions.  
            carWidthArr = zeros(1,length(carsArr));
            for i=1:length(carsArr)
                obj = carsArr(i);
                carWidthArr(i) = obj.state(1);
            end
            [~, I] = sort(carWidthArr, 'descend');
            sortedCars = carsArr(I);
        end
    end
end

