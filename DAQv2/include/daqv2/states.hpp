#pragma once

#include "daqv2/fsm.hpp"

class IdleState : public EngineState {
public:
  void on_enter(SystemParameters& params) override;
  bool update(SystemParameters& params) override;
  void on_exit(SystemParameters& params) override;
  std::string get_name() const override;
  bool validate_entry(const SystemParameters& params) const override;
  std::vector<std::string> get_allowed_transitions() const override;
};
