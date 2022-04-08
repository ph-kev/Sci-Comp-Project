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

        function [leftSafe, rightSafe] = checkSafe(obj, carArr, maxLane, b_safe, gamma)
            % \brief Check if it is safe to switch to the right or left
            % lane.
            % \param obj, car which is getting updated.
            % \param carArr, array of cars sorted by distance.
            % \param maxLane, maximum number of lanes on the road.
            % \param b_safe, limit for safe deacceleration. 
            % \param gamma, speed difference sensitivity. 
            % \returns Array of lanes that are safe to switch into.
            laneNum = obj.lane;
            rightLaneNum = laneNum + 1;
            leftLaneNum = laneNum - 1;
            [~, backRight,~] = obj.findCar(carArr, rightLaneNum);
            [~, backLeft,~] = obj.findCar(carArr, leftLaneNum);
            if rightLaneNum == maxLane + 1
                rightSafe = false;
            else 
                if isempty(backRight)
                    rightSafe = true;
                else
                    safeValue = obj.safeCriterion(backRight,b_safe,gamma);
                    s_f_new = obj.state(1) - backRight.state(1) - obj.width;
                    if s_f_new > safeValue 
                        rightSafe = true;
                    else 
                        rightSafe = false; 
                    end
                end
            end
            if leftLaneNum == 0 
                leftSafe = false;
            else
                if isempty(backLeft)
                    leftSafe = true;
                else
                    safeValue = obj.safeCriterion(backLeft,b_safe,gamma);
                    s_f_new = obj.state(1) - backLeft.state(1) - obj.width;
                    if s_f_new > safeValue 
                        leftSafe = true;
                    else 
                        leftSafe = false; 
                    end
                end
            end
        end

        function findIncentive = findIncentive(obj, carArr, leftSafe, rightSafe)
            % \brief Find the utility of changing lane.
            % \param obj, car where we are finding the utlity of.
            % \param carArr, array of cars sorted by distance.
            % \param laneArr, array of lanes that we can switch to.
            % \returns Best possible lane to switch to.
            laneNum = obj.lane;
            rightLaneNum = laneNum + 1;
            leftLaneNum = laneNum - 1;
            [frontCarRight, ~, ~] = obj.findCar(carArr, rightLaneNum);
            [frontCarLeft, ~, ~] = obj.findCar(carArr, leftLaneNum);
            [frontCarOld, ~, ~] = obj.findCar(carArr, laneNum);
            if isempty(frontCarRight)
                frontCarRight = obj;
            end
            if isempty(frontCarLeft)
                frontCarLeft = obj;
            end
            if isempty(frontCarOld)
                frontCarOld = obj;
            end
            if leftSafe == 1
                incentive = obj.incentiveCriterion(frontCarOld,frontCarLeft, delta_a, a_bias, 1, gamma);
                % TODO
            else 
                leftIncentive = false;
            end
            if rightSafe == 1
                incentive = obj.incentiveCriterion(frontCarOld,frontCarRight, delta_a, a_bias, 0, gamma);
                % TODO
            else 
                rightIncentive = false;
            end 
            % logic for dealing with rightIncentive and leftIncentive
        end

        function [front,back,index] = findCar(obj, carArr, laneNum)
            % \brief Find the next and back cars in the lane that the car
            % is switching to.
            % \param obj, car changing to laneNum.
            % \param carsArr, array of cars sorted by distance.
            % \param laneNum, index of lane we are switching to.
            % \return front, the car in front of obj after lane change.
            %         back, the car behind of obj after lane change.
            %         index, index of obj in carArr 
            front = Vehicle.empty(1,0);
            back = Vehicle.empty(1,0);
            carNum = obj.number;
            index = 0;
            for i=1:length(carArr)
                if carArr(i).number == carNum
                    index = i;
                end
            end
            frontCars = carArr(1:index - 1);
            backCars = carArr(index + 1:end);

            if isempty(frontCars)
            else
                for i=length(frontCars):1
                    if frontCars(i).lane == laneNum
                        front = frontCars(i);
                        break
                    end
                end
            end

            if isempty(backCars)
            else
                for i=1:length(backCars)
                    if backCars(i).lane == laneNum
                        back = backCars(i);
                        break
                    end
                end
            end
        end
        function safeCriterion = safeCriterion(obj,backCar,b_safe,gamma)
            % \brief Safety criterion for lane changing.
            % \param obj, car doing lane change.
            % \param backCar, car behind the car that is doing a lane.
            % \param b_safe, b_safe, limit for safe deacceleration.
            % \param gamma, speed difference sensitivity.
            % \returns Value of the safety criterion.
            v_f_hat = backCar.state(2);
            v_a = obj.state(2);
            tau = backCar.params(1);
            value = v_f_hat - tau * b_safe + tau * gamma * (v_f_hat - v_a);
            safeCriterion = backCar.se(value); % not sure if this is right, don't know what car we should use se on
        end

        function incentiveCriterion = incentiveCriterion(obj,frontCarOld,frontCarNew, delta_a, a_bias, leftLane, gamma)
            % \brief Incentive condition for lane changing.
            % \param obj, car doing lane change.
            % \param frontCarOld, car in front of obj before lane change.
            % \param frontCarNew, car in front of obj after lane change.
            % \param delta_a, changing threshold.
            % \param a_bias, keep-left directive. 
            % \param leftLane, if 1, then the car is switching to the left
            % lane and if 0, then the car is switching to the right lane.
            % \returns Value of the incentive criterion.
            if leftLane == 0  %  check if we are in the fast lane 
                a_bias = -1 * a_bias;
            end
            s_a = frontCarOld.state(1) - obj.state(1) - frontCarOld.width;
            v_l = frontCarOld.state(2);
            v_l_hat = frontCarNew.state(1);
            tau = frontCarNew.params(1); 
            value = obj.v_opt(s_a) + tau * (delta_a + a_bias + gamma * (v_l - v_l_hat)); % not sure if v_opt is correct here
            incentiveCriterion = obj.se(value); % not sure if this is right, don't know what car we should use se on
        end

        function se = se(obj,v)
            % \brief Compute the inverse of v_opt. 
            % \param obj, car.
            % \param value to substitute in the inverse of v_opt.
            % \return se, value substituted in the verse of v_opt.
            if v < 0
                se = v;  % not sure if we want this kind of behavior (don't know if it should be 0 or negative)
            elseif v > obj.params(2)
                se = obj.params(2);
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
    end
end

