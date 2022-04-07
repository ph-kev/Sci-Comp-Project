classdef Vehicle<handle
    % VEHICLE Summary of this class goes here

    properties
        number % Number of car.
        width % Length of car.
        state % Position, velocity, and acceleration.
        params % Adaptation time, v_desired, transition_width, form_factor, time_gap, min_gap.
        lane % Lane number.
        isTrafficLight
    end

    methods
        function obj = Vehicle(n, x_0, v_0, a_0, lane, width, tl, varargin)
            % VEHICLE Construct an instance of this class
            % \brief Find the optimal velocity of the car.
            % \param obj, car.
            % \param s, gap of the car and the leading car.
            % \returns Vehicle's optimal velocity.
            if nargin > 0 % Check the number of parameters.
                obj.number = n;
                obj.width = width;
                obj.state = [x_0, v_0, a_0];
                obj.lane = lane;
                obj.isTrafficLight = tl;
                % Optional paramters.
                obj.params = [0.65, 33.3, 15, 1.5, 1.4, 3];
                if (length(varargin) > length(obj.params))
                    error('Too many input parameters');
                end
                varargin = cell2mat(varargin);
                for i=1:length(varargin)
                    obj.params(i) = varargin(i);
                end
            end
        end
        
        % Add Constructor for traffic light length = 0 and desired v = 0

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
        
        function changeLane = changeLane(obj)
            % \brief Check and do lane change if possible.
            % \param obj, car which is doing the lane change.
            % \returns Updated car after lane change.
        end

        function checkSafe = checkSafe(obj, carArr)
            % \brief Check if it is safe to switch to the right or left
            % lane.
            % \param obj, car which is getting updated.
            % \param carArr, array of cars sorted by distance.
            % \returns Array of lanes that are safe to switch into.
        end
        
        function findIncentive = findIncentive(obj, carArr, laneArr)
            % \brief Find the utility of changing lane.
            % \param obj, car where we are finding the utlity of.
            % \param carArr, array of cars sorted by distance.
            % \param laneArr, array of lanes that we can switch to.
            % \returns Best possible lane to switch to.
        end

        function [front,back] = findCar(obj, carArr, laneNum)
            % \brief Find the next and back cars in the lane that the car
            % is switching to. 
            % \param obj, car doing the lane change.
            % \param carArr, array of cars sorted by distance.
            % \param laneNum, index of lane we are switching to
            % \returns The car in front and back of it if the car does a
            %          lane change. 
            front = Vehicle.empty(1,1);
            back = Vehicle.empty(1,1);
            carPos = obj.state(1);
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

        function safeCriterion = safeCriterion(v_f_hat, v_a)
            % \brief Safety criterion for lane changing.
            % \param v_f_hat, velocity of the follower after lane change.
            % \parm v_a, velocity of car doing the lane change.
            % \returns Value of the safety criterion. 
        end

        function incentiveCriterion = incentiveCriterion(s_a, v_l, v_l_hat)
            % \brief Incentive condition for lane changing.
            % \param s_a, gap of the car doing lane change. 
            % \param v_l, velocity of the leader car before lane change. 
            % \param v_l_hat, velocity of the leader car after lane change.
            % \returns Value of the incentive criterion.
        end
    end
end

