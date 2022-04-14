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
                obj.params = [5, 33.3, 15, 1.5, 1.4, 3];
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


        function newlane = changeLane(obj, index, carArr, maxLane, b_safe, delta_a, a_bias, gamma, x_destination)
            % \brief Finds best Lane to change to. 
            % \param obj, car which is getting updated.
            % \param index, the index of obj in carArr
            % \param carArr, array of cars sorted by distance.
            % \param maxLane, maximum number of lanes on the road.
            % \param b_safe, limit for safe deacceleration. 
            % \param gamma, speed difference sensitivity. 
            % \param x_destination, the distance to the destination
            % \returns The best lane to switch to.
            laneNum = obj.lane;
            
            [frontRight_v, frontRight_gap, backRight_v, backRight_gap] = Vehicle.findGaps(carArr, index, laneNum + 1, x_destination);
            [frontLeft_v, frontLeft_gap, backLeft_v, backLeft_gap] = Vehicle.findGaps(carArr, index, laneNum - 1, x_destination);
            [front_v, front_gap, ~, ~] = Vehicle.findGaps(carArr, index, laneNum, x_destination);

            if laneNum == maxLane
                rightSafe = false;
                rightIncentive = false;
            else 
                safeValue = obj.safeCriterion(backRight_v, obj.state(2) ,b_safe,gamma);
                if backRight_gap > safeValue
                    rightSafe = true;
                else
                    rightSafe = false;
                end
                
                advValue = obj.incentiveCriterion(front_gap, front_v, frontRight_v, delta_a, a_bias, false, gamma);
                if frontRight_gap > advValue
                   rightIncentive = true; 
                else
                    rightIncentive = false;
                end
            end
            if laneNum == 1
                leftSafe = false;
                leftIncentive = false;
            else
                safeValue = obj.safeCriterion(backLeft_v, obj.state(2), b_safe, gamma);
                if backLeft_gap > safeValue
                    leftSafe = true;
                else
                    leftSafe = false;
                end
                
                advValue = obj.incentiveCriterion(front_gap, front_v, frontLeft_v, delta_a, a_bias, true, gamma);
                if frontLeft_gap > advValue
                   leftIncentive = true; 
                else
                    leftIncentive = false;
                end
            end
            
            if leftSafe && leftIncentive 
                newlane = laneNum-1;
            elseif rightSafe && rightIncentive
                newlane = laneNum+1;
            else
                newlane = laneNum;
            end
            
            obj.lane = newlane;
        end

        function safeCriterion = safeCriterion(obj, v_f_hat, v_a, b_safe,gamma)
            % \brief Safety criterion for lane changing.
            % \param obj, car doing lane change.
            % \param backCar, car behind the car that is doing a lane.
            % \param b_safe, b_safe, limit for safe deacceleration.
            % \param gamma, speed difference sensitivity.
            % \returns Value of the safety criterion.
            tau = obj.params(1);
            value = v_f_hat - tau * b_safe + tau * gamma * (v_f_hat - v_a);
            safeCriterion = obj.se(value); % not sure if this is right, don't know what car we should use se on
        end

        function incentiveCriterion = incentiveCriterion(obj, s_a, v_l, v_l_hat, delta_a, a_bias, leftLane, gamma)
            % \brief Incentive condition for lane changing.
            % \param obj, car doing lane change.
            % \param delta_a, changing threshold.
            % \param a_bias, keep-left directive. 
            % \param leftLane, if 1, then the car is switching to the left
            % lane and if 0, then the car is switching to the right lane.
            % \returns Value of the incentive criterion.
            if leftLane == 0  %  check if we are in the fast lane 
                a_bias = -1 * a_bias;
            end
            tau = obj.params(1); 
            value =  tau * (delta_a + a_bias + gamma * (v_l - v_l_hat)); % not sure if v_opt is correct here
            incentiveCriterion = s_a + obj.se(value); % not sure if this is right, don't know what car we should use se on
        end

        function se = se(obj,v)
            % \brief Compute the inverse of v_opt. 
            % \param obj, car.
            % \param value to substitute in the inverse of v_opt.
            % \return se, value substituted in the verse of v_opt.
            if v < 0
                se = 0;  % not sure if we want this kind of behavior (don't know if it should be 0 or negative)
            elseif v > obj.params(2)
                se = v;  % might need to change this back to obj.params(2)
            else 
                s_0 = obj.params(6);
                T = obj.params(5);
                se = s_0 + T * v;
            end
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
        
        function [front_v, front_gap ,back_v, back_gap] = findGaps(carArr, index, laneNum, x_destination)
            % \brief Find the next and back cars in the lane that the car
            % is switching to.
            % \param obj, car changing to laneNum.
            % \param carsArr, array of cars sorted by distance.
            % \param laneNum, index of lane we are searching.
            % \return front_v, the velocity of the car in front in the lane
            %         specified by laneNum.
            %         back_v, the velocity of the car in the back.
            %         front_gap, the gap between the car at index and the
            %         car in front.
            next = index;
            previous = index;
            
            while(1)%Loop that finds the next car
                if next < 2 %If there are no cars in front
                    next = index;
                    break;
                else
                    next = next-1;
                end

                if carArr(next).lane == laneNum
                    break;
                end
            end
            % set leading vehicle velocity to speed limit if no cars is in front
            if index == next  % First car.
                % Find gap using first car's position and destination.
                front_gap = x_destination - carArr(index).state(1);
                front_v = 0;
            else % Cars after the first car.
                % Compute gap and use velocity using the car in front of it.
                front_gap = carArr(next).state(1) - carArr(next).width - carArr(index).state(1);
                front_v = carArr(next).state(2);
            end
    
            while(1)% finds the previous car
                if previous >= length(carArr) %If there are no cars in front
                    previous = index;
                    break;
                else
                    previous = previous + 1;
                end

                if carArr(previous).lane == laneNum
                    break;
                end
            end
            if index == previous  % For the last car, following v is 0.
                back_v = 0;
                back_gap = carArr(index).state(1);
            else % Cars after the last car.
                back_v = carArr(previous).state(2);
                back_gap = carArr(index).state(1) - carArr(previous).state(1) - carArr(index).width;
            end
            
        end
        
    end
end

