#include "SensorFSM.h"

// These are provided externally (mocked in tests, real on Arduino)
bool serverHeartbeatReceived();
unsigned long getTimeMs();

static const unsigned long HEARTBEAT_TIMEOUT_MS = 5000;

SensorFSM::SensorFSM()
    : state(SensorState::INIT), lastHeartbeat(0) {}

void SensorFSM::update() {
    switch (state) {
        case SensorState::INIT:
            lastHeartbeat = getTimeMs();
            state = SensorState::CHECK_SERVER;
            break;

        case SensorState::CHECK_SERVER:
            if (serverHeartbeatReceived()) {
                lastHeartbeat = getTimeMs();
                state = SensorState::RUNNING;
            } else if (getTimeMs() - lastHeartbeat > HEARTBEAT_TIMEOUT_MS) {
                state = SensorState::SERVER_DOWN;
            }
            break;

        case SensorState::RUNNING:
            if (serverHeartbeatReceived()) {
                lastHeartbeat = getTimeMs();
            } else if (getTimeMs() - lastHeartbeat > HEARTBEAT_TIMEOUT_MS) {
                state = SensorState::SERVER_DOWN;
            }
            break;

        case SensorState::SERVER_DOWN:
            // stay here or implement recovery
            break;
    }
}

SensorState SensorFSM::getState() const {
    return state;
}
