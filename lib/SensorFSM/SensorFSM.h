#pragma once

enum class SensorState {
    INIT,
    CHECK_SERVER,
    RUNNING,
    SERVER_DOWN
};

class SensorFSM {
public:
    SensorFSM();

    void update();
    SensorState getState() const;

private:
    SensorState state;
    unsigned long lastHeartbeat;
};
