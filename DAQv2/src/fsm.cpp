#include "daqv2/fsm.hpp"

#include <algorithm>
#include <iostream>
#include <utility>

EngineFSM::EngineFSM() : current_state("") {}

void EngineFSM::register_state(const std::string &state_name,
                               std::shared_ptr<EngineState> state)
{
    if (states.find(state_name) != states.end())
    {
        std::cerr << "Warning: State '" << state_name << "' already registered\n";
        return;
    }
    states[state_name] = std::move(state);

    if (current_state.empty())
    {
        current_state = state_name;
    }
}

void EngineFSM::update(SystemParameters &params)
{
    if (states.find(current_state) == states.end())
    {
        return;
    }

    auto current = states[current_state];
    current->update(params);

    if (current->is_automatic())
    {
        std::string next_state_name = current->get_next_state_name();
        if (!next_state_name.empty() && states.find(next_state_name) != states.end())
        {
            if (can_enter_state(next_state_name, params))
            {
                transition_to_state(next_state_name, params);
            }
        }
    }
}

bool EngineFSM::request_transition(const std::string &target_state,
                                    SystemParameters &params)
{
    if (states.find(target_state) == states.end())
    {
        std::cerr << "Error: Target state '" << target_state << "' not registered\n";
        return false;
    }

    if (!can_enter_state(target_state, params))
    {
        std::cerr << "Error: Cannot enter state '" << target_state
                  << "' - validation criteria not met\n";
        return false;
    }

    transition_to_state(target_state, params);
    return true;
}

void EngineFSM::transition_to_state(const std::string &state_name,
                                    SystemParameters &params)
{
    if (states.find(current_state) != states.end())
    {
        states[current_state]->on_exit(params);
        std::cout << "Exiting state: " << states[current_state]->get_name() << "\n";
    }

    current_state = state_name;
    states[current_state]->on_enter(params);
    std::cout << "Entering state: " << states[current_state]->get_name() << "\n";
}

std::string EngineFSM::get_current_state() const
{
    return current_state;
}

bool EngineFSM::can_enter_state(const std::string &state_name,
                                const SystemParameters &params) const
{
    if (states.find(state_name) == states.end())
    {
        return false;
    }
    auto allowed = states.at(current_state)->get_allowed_transitions();
    if (std::find(allowed.begin(), allowed.end(), state_name) == allowed.end())
    {
        return false;
    }
    return states.at(state_name)->validate_entry(params);
}

std::vector<std::string> EngineFSM::get_available_transitions(
    const SystemParameters &params) const
{
    std::vector<std::string> available;

    if (states.find(current_state) == states.end())
    {
        return available;
    }

    auto allowed = states.at(current_state)->get_allowed_transitions();
    for (const auto &target_state : allowed)
    {
        if (can_enter_state(target_state, params))
        {
            available.push_back(target_state);
        }
    }

    return available;
}
