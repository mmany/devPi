#include <iostream>
#include <fstream>
#include <serial/serial.h> //needs previus installed serial library and its dependency by William Woodall see http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html
#include <string>
#include <stdint.h>
#include <iomanip>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <chrono>
#include <ctime>
#include <sys/shm.h>
#include <sys/mman.h>
#include <wiringPi.h>
#include <sstream>
#include <semaphore.h>

// LOG File for General Information about the System (for all participating processes)
#define INFO_LOG "/home/pi/instrumentation_logs/INFO_LOG.txt"

// ECU preset
#define ECU_BAUD 112000
#define ECU_PORT "/dev/ECU"
#define ECU_LOG_LOCATION "/home/pi/instrumentation_logs/ECU/"
#define ECU_LOG_PREFIX "ECULOG_"
#define ECU_SLAVE_ADDRESS "1"
#define ECU_END_BIT "\r" // Carriage return

// shared memory names
#define SH_MEM_NAME_ECU "ECUshmem"



// struct for ECU shared memory
typedef struct ECUdata{
    // TO BE DEFINED
    unsigned long data;
}

string ECULOGfile;

string ecustartmatch;

void* shmem_ptr_ECU = nullptr;

using namespace std;
using namespace serial;
using namespace std::chrono;

std::chrono::time_point timestamp;


bool HSok = false;
bool LOGbool = false;

// creates shared memory, which can be accesed from all processes that created sh.mem. with same name
void* create_shared_mem(const int size, const char* name);

// send command to ECU
void sendtoECU(const unsigned char send[], int size);
// parse incoming data from serial
bool ecudata(void);
// parse incoming message from serial
bool ecumessage(void);
// first instance to assure incoming serial data is aligned to begining of data or message
bool ECUparse(void);
// LOGGING
void LOGinfo(int casenmbr);
// function creating the shared memory for ECU data
void* create_shared_mem(const int size, const char* name){

