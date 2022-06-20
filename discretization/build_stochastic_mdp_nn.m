% build_stochastic_mdp_nn: Function implementing the Nearest Neighbour
%                          approach for creating a stochastic MDP
%
% Inputs:
%       world:                  A structure containing basic parameters for
%                               the mountain car problem
%       T:                      Transition model with elements initialized
%                               to zero
%       R:                      Expected reward model with elements
%                               initialized to zero
%       num_samples:            Number of samples to use for creating the
%                               stochastic model
%
% Outputs:
%       T:                      Transition model with elements T{a}(s,s')
%                               being the probability of transition to 
%                               state s' from state s taking action a
%       R:                      Expected reward model with elements 
%                               R{a}(s,s') being the expected reward on 
%                               transition from s to s' under action a
%
% --
% Control for Robotics
% AER1517 Spring 2022
% Assignment 4
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistant: 
% SiQi Zhou
% siqi.zhou@robotics.utias.utoronto.ca
% Lukas Brunke
% lukas.brunke@robotics.utias.utoronto.ca
% Adam Hall
% adam.hall@robotics.utias.utoronto.ca
%
% --
% Revision history
% [20.03.07, SZ]    first version

function [T, R] = build_stochastic_mdp_nn(world, T, R, num_samples)
    % Extract states and actions
    STATES = world.mdp.STATES;
    ACTIONS = world.mdp.ACTIONS;
    rng(42)
    % Dimensions
    num_states = size(STATES, 2);
    num_actions = size(ACTIONS, 2);

    % Loop through all possible states
    for state_index = 1:1:num_states
        cur_state = STATES(:, state_index);
        fprintf('building model... state %d\n', state_index);

        % Apply each possible action
        for action_index = 1:1:num_actions
            action = ACTIONS(:, action_index);

            % [TODO] Build a stochastic MDP based on Nearest Neighbour
            % Note: The function 'nearest_state_index_lookup' can be used
            % to find the nearest node to a countinuous state
            
            % Propagate forward
            [next_state, reward, ~] = world.one_step_model(world, ...
                cur_state, action);
            % Set distribution variables for added noise
            mu_1 = 0;
            mu_2 = 0;
            sigma_1 = 0.0006;
            sigma_2 = 0.0004;
            for samples = 1:1:num_samples
                % Add noise to state to find continous state
                noise_1 = normrnd(mu_1,sigma_1);
                noise_2 = normrnd(mu_2,sigma_2);
                next_state(1) = next_state(1) + noise_1;
                next_state(2) = next_state(2) + noise_2;
                % Find nearest neighbor
                next_state_index = ...
                    nearest_state_index_lookup(STATES, next_state);

                % Update transition and reward models
                T{action_index}(state_index, next_state_index) = ...
                    T{action_index}(state_index, next_state_index) ...
                    + 1/num_samples;
                if next_state_index <= 380
                    R{action_index}(state_index, next_state_index) = -1;
                else
                    R{action_index}(state_index, next_state_index) = 10;
                end
            end
        end
    end
end

