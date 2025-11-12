#include "daqv2/states.hpp"

#include <iostream>

void IdleState::on_enter(SystemParameters& p) {
    p.SOL_PV=p.SOL_FUP=p.SOL_FV=p.SOL_OUP=p.SOL_OV=p.SOL_FDP=false;
    p.MOT_ODP=false;
    std::cout << "  Idle: valves closed\n";
}

bool IdleState::update(SystemParameters&) {
    return false;
}

void IdleState::on_exit(SystemParameters&) {}

std::string IdleState::get_name() const {
    return "Idle";
}

bool IdleState::validate_entry(const SystemParameters&) const {
    return true;
}

std::vector<std::string> IdleState::get_allowed_transitions() const {
    return {"Idle"};
}
