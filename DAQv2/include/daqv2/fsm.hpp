#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

struct SystemParameters
{
    // Pressures (psi)
    float PT_HP = 0.0f;
    float PT_LP = 0.0f;
    float PT_F = 0.0f;
    float PT_O = 0.0f;

    float RTD_O1 = 0.0f;
    float RTD_O2 = 0.0f;
    float RTD_O3 = 0.0f;

    float PT_I = 0.0f;
    float TC_I = 0.0f;
    float PT_C1 = 0.0f;
    float TC_C1 = 0.0f;
    float TC_C3 = 0.0f;
    float TC_C2 = 0.0f;
    float PT_C2 = 0.0f;
    float TC_C4 = 0.0f;

    // Solenoid states (true = open/energized, false = closed/de-energized)
    bool SOL_PV = false;
    bool SOL_FUP = false;
    bool SOL_FV = false;
    bool SOL_OUP = false;
    bool SOL_OV = false;
    bool SOL_FDP = false;
    bool MOT_ODP = false;

    bool ROT_MF = false;
    bool ROT_MO = false;
};

class EngineState
{
public:
    virtual ~EngineState() = default;
    virtual void on_enter(SystemParameters &params) = 0;
    virtual bool update(SystemParameters &params) = 0;
    virtual void on_exit(SystemParameters &params) = 0;
    virtual std::string get_name() const = 0;
    virtual bool validate_entry(const SystemParameters &params) const = 0;
    virtual std::vector<std::string> get_allowed_transitions() const { return {}; }
    virtual std::string get_next_state_name() const { return ""; }
    virtual bool is_automatic() const { return false; }
};

class EngineFSM
{
public:
    EngineFSM();

    void register_state(const std::string &state_name, std::shared_ptr<EngineState> state);
    bool request_transition(const std::string &target_state, SystemParameters &params);
    void update(SystemParameters &params);
    std::string get_current_state() const;
    std::vector<std::string> get_available_transitions(const SystemParameters &params) const;

private:
    void transition_to_state(const std::string &state_name, SystemParameters &params);
    bool can_enter_state(const std::string &state_name, const SystemParameters &params) const;

    std::map<std::string, std::shared_ptr<EngineState>> states;
    std::string current_state;
};

