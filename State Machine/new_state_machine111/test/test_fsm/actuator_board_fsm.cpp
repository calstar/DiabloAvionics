#include <unity.h>
#include "SensorFSM.h"
#include <cstdlib>
#include <iostream>

// ---------------- Mock dependencies ----------------
static bool serverHeartbeat = true;
static bool configPacket = true;
int boardID = 1234; // Example board ID
static const int HEARTBEAT_TIMEOUT_MS = 5000; // 5 seconds timeout example
static const std::string SERVER_ADDRESS = "SERVER_ADDRESS";
static const std::string ACTUATOR_CONTROLLER_ADDRESS = "ACTUATOR_CONTROLLER_ADDRESS";
const int IPAddress = 0; // Example IP address (recieve during packet config)
const bool necessaryForAbort = true; //example condition for abort
enum class BoardState{
    inactive,
    active,
    disconnected,
    abort
};


BoardState myBoardState = BoardState::inactive;

void setUp(void) {
    // called before each test
}
void tearDown(void) {
    // called after each test
}

bool heartbeatTimedOut(int currTime, int lastHeartbeatTime) {
    return (currTime - lastHeartbeatTime) > HEARTBEAT_TIMEOUT_MS;
}
void myTest() {
    TEST_ASSERT_TRUE(true); // Example test that always passes
}
void sendHeartbeat(int boardID, BoardState boardState, std::string destination) {
    (void)boardID;// Mock implementation of sending heartbeat w/ ID
}
bool serverHeartbeatReceived() {
    return serverHeartbeat;
}
bool configPacketReceived() {
    return configPacket; // Mock implementation
}
void storeAddress(){
    //stores server address in server heartbeat
}
void delay(int ms){
    //mock delay function
}
bool testConnection(){
    //mock connection test function
    return false; //assume connection failed for example
}
void streamData(int IPAddress){
    (void)IPAddress; //mock stream data function
}
bool abortClearPacket(){
    return true; //mock abort clear packet received, returns if abort clear packet is received
}
bool abortPacket(){
    return true; //mock abort packet received, returns if abort packet is received
}
void connectionLoss(BoardState myBoardstate, int boardID);
void mainloop(BoardState myBoardstate);//goes in update()
void provideAbortData(BoardState myBoardstate, int boardID, int IPAddress, std::string destination);

// ---------------- Test runner ----------------
int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    //Power on
    UNITY_BEGIN(); // begins Unity testing framework
    RUN_TEST(myTest); //example test, it calls the function which has the assertion test
    myBoardState = BoardState::inactive;
    
    //Waiting for Server
    //send heartbeats including board ID until it recieves server's heartbeat response or timeout
    while(serverHeartbeatReceived() == false){
        sendHeartbeat(boardID, myBoardState, SERVER_ADDRESS);
    }

    storeAddress(); //store server address in server heartbeat
    //Waiting for Config
    while(configPacketReceived() == false){
        //just wait to recieve?
    }
    
    //everything is set up, board is active
    myBoardState = BoardState::active;
    //Active loop
    mainloop(myBoardState);
    
    

    return UNITY_END(); //need this to exit out?
}

void mainloop(BoardState myBoardstate, int boardID){
   //send heartbeats w state being active
   //store data in config packet
   //stream sensor packet to server
   //actuator boards recieve and execute actuation packets from server


}
void connectionLoss(BoardState myBoardstate, int boardID){
    //handle connection loss
     myBoardstate = BoardState::disconnected;
            
            //wait for X seconds
            delay(5000); //5 seconds delay example
            if(testConnection() == true){; //check connection
                myBoardState = BoardState::active; //back to normal
                mainloop(myBoardstate, boardID); //return to main loop
            }
            else{ //connection failed
                myBoardstate = BoardState::abort;
                sendHeartbeat(boardID, myBoardstate, ACTUATOR_CONTROLLER_ADDRESS); //send to actuator controller
                provideAbortData(myBoardstate, boardID, IPAddress, ACTUATOR_CONTROLLER_ADDRESS); //example IP address 0
            }
}
void provideAbortData(BoardState myBoardstate, int boardID, int IPAddress, std::string destination){
    myBoardState = BoardState::abort;
    sendHeartbeat(boardID, myBoardstate, destination);
    streamData(IPAddress);

    if(abortClearPacket() == true){
        mainloop(myBoardstate, boardID); //return to main loop
    }
}
// sample starter code
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