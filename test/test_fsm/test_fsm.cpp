#include <unity.h>
#include "SensorFSM.h"

// ---------------- Mock dependencies ----------------
static bool mockHeartbeat = false;
static unsigned long fakeTime = 0;

void setUp(void) {
    // called before each test
}

void tearDown(void) {
    // called after each test
}

void myTest() {
    TEST_ASSERT_TRUE(true); // Example test that always passes
}
// ---------------- Test runner ----------------
int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN(); // begins Unity testing framework
    RUN_TEST(myTest); //example test, it calls the function which has the assertion test
    
    //Waiting for Server

    //Waiting for Config


    //Active loop
    while(true){
        //to be implemented
        break;
    }



    
    return UNITY_END(); //need this to exit out?
}



//Chat's sample starter code
/*
#include <unity.h>
#include "SensorFSM.h"

// ---------------- Mock dependencies ----------------
static bool mockHeartbeat = false;
static unsigned long fakeTime = 0;

void setUp(void) {
    // called before each test
}

void tearDown(void) {
    // called after each test
}


bool serverHeartbeatReceived() {
    return mockHeartbeat;
}

unsigned long getTimeMs() {
    return fakeTime;
}

// ---------------- Tests ----------------
void test_initial_state() {
    SensorFSM fsm;
    TEST_ASSERT_EQUAL(SensorState::INIT, fsm.getState());
}

void test_server_timeout() {
    SensorFSM fsm;

    mockHeartbeat = false;
    fakeTime = 6000;

    fsm.update(); // INIT -> CHECK_SERVER
    fsm.update(); // timeout

    TEST_ASSERT_EQUAL(SensorState::SERVER_DOWN, fsm.getState());
}

// ---------------- Test runner ----------------
int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(test_initial_state);
    RUN_TEST(test_server_timeout);
    return UNITY_END();
}


*/