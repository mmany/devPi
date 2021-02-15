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

// General LOG presets
#define LOG_SUFFIX "txt"
#define MAX_LOG_FILE 999

// LOG File for General Information about the System (for all participating processes)
#define INFO_LOG "/home/pi/instrumentation_logs/INFO_LOG.txt"

// GPIO Pin for GPS time syncroutine
#define INTERUPT_PIN 1


// IMU preset
#define IMU_BAUD 1000000
#define IMU_PORT "/dev/IMU"
#define DATA_PACKAGE_SIZE 29
#define MESSAGE_PACKAGE_SIZE 5
#define IMU_LOG_LOCATION "/home/pi/instrumentation_logs/IMU/"
#define IMU_LOG_PREFIX "IMULOG_"

// GPS preset
#define GPS_BAUD 38400
#define GPS_PORT "/dev/serial0"
#define PVT_PACKAGE_SIZE 96
#define SYNC_PACKAGE_SIZE 32
#define ACK_PACKAGE_SIZE 6
#define NAK_PACKAGE_SIZE 6
#define GPS_LOG_LOCATION "/home/pi/instrumentation_logs/GPS/"
#define GPS_LOG_PREFIX "GPSLOG_"

// NANO (PPM Signal and Fuelflow) preset
#define NANO_BAUD 115200
#define NANO_PORT "/dev/NANO"
#define NANO_DATA_PACKAGE_SIZE 39
#define NANO_MESSAGE_PACKAGE_SIZE 5
#define NANO_PPM_CHANNELS 13
#define NANO_LOG_LOCATION "/home/pi/instrumentation_logs/NANO/"
#define NANO_LOG_PREFIX "NANOLOG_"

// ADP preset
#define ADP_BAUD 500000
#define ADP_PORT "/dev/ADP"
#define ADP_DATA_PACKAGE_SIZE 13
#define ADP_MESSAGE_PACKAGE_SIZE 5
#define ADP_LOG_LOCATION "/home/pi/instrumentation_logs/ADP/"
#define ADP_LOG_PREFIX "ADPLOG_"

// THRust sensor preset
#define THR_BAUD 500000
#define THR_PORT "/dev/MEGA"
#define THR_DATA_PACKAGE_SIZE 29
#define THR_MESSAGE_PACKAGE_SIZE 5
#define THR_LOG_LOCATION "/home/pi/instrumentation_logs/THRUST/"
#define THR_LOG_PREFIX "THRUSTLOG_"


// shared memory names
#define SH_MEM_NAME_IMU "IMUshmem"
#define SH_MEM_NAME_MSG "MSGshmem" //Process Message Protocoll in form of float numbers
#define SH_MEM_NAME_GPS "GPSshmem"
#define SH_MEM_NAME_ADP "ADPshmem"
#define SH_MEM_NAME_NANO "NANOshmem"
#define SH_MEM_NAME_THR "THRshmem"

// semaphore for message shared memory between processes
#define MSG_SEM_NAME "/msgsem"

using namespace std;
using namespace serial;
using namespace std::chrono;

// structs for incoming data
typedef struct IMUdata{
  unsigned long IMUtime;
  float variables[6];
};

typedef struct GPSSYNC{
  unsigned char ch;
  unsigned char flags;
  unsigned short count;
  unsigned short wnR;
  unsigned short wnF;
  unsigned long towMsR;
  unsigned short towSubMsR;
  unsigned long towMsF;
  unsigned short towSubMsF;
  unsigned short accEst;
};

typedef struct GPSdata{ // see ublox NAV-PVT message, for data explenation
  unsigned long iTOW;
  unsigned short year;
  unsigned char month;
  unsigned char day;
  unsigned char hour;
  unsigned char min;
  unsigned char sec;
  unsigned char valid;
  unsigned long tAcc;
  long nano;
  unsigned char fixType;
  unsigned char flags;
  unsigned char flags2;
  unsigned char numSV;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
  long velN;
  long velE;
  long velD;
  unsigned long gSpeed;
  long heading;
  unsigned long sAcc;
  unsigned long headingAcc;
  unsigned short pDOP;
  unsigned char reserved1;
  long headVeh;
  short magDec;
  short magAcc;
};

typedef struct NANOdata{
  unsigned long NANOtime;
  float vdot;
  float vges;
  short int channelvalue[NANO_PPM_CHANNELS];
};

typedef struct ADPdata{
  unsigned long ADPtime;
  float P_dyn;
  float P_static;
};

typedef struct THRUSTdata{
  unsigned long THRtime;
  float temp1;
  float temp2;
  float force1;
  float force2;
  unsigned long raw1;
  unsigned long raw2;
};

// translation of binary data to float data
typedef union binaryfloat{
  float msg;
  char binary[4];
};

// translation of binary data to unsigned long data
typedef union binaryunsignedlong{
  unsigned long ul;
  char binary[4];
};

// system time points for synchronisation of different Sensors
high_resolution_clock::time_point tIMU;
high_resolution_clock::time_point tNANO;
high_resolution_clock::time_point tADP;
high_resolution_clock::time_point tTHR;
high_resolution_clock::time_point tGPS;
std::chrono::duration<double, std::milli> tdiff; //time duration to relate the timepoints relative to another

bool processbool = true; //main loop
bool GPSvalidsolution = false; // valid GPS fix
bool LOGbool = false; //active Loging
bool IMUsync = false; //received IMU start up time
bool NANOsync = false; //received NANO start up time
bool ADPsync = false; //received ADP start up time
bool THRsync = false; //received THRUST start up time
bool GPSsync = false; //received GPS interrupt time
bool sentyaw = false; //received YAW initialization from IMU and sent it to Strapdown Process through Message Protocoll
bool Ack = false; //GPS setup Acknowlaged from receiver
bool Nak = false; //GPS setup not Acknowlaged from receiver

//received start up time as unsigned long
unsigned long IMU_sutime_ul;
unsigned long NANO_sutime_ul;
unsigned long ADP_sutime_ul;
unsigned long THR_sutime_ul;
//received start up time as double
double IMU_sutime;
double NANO_sutime;
double ADP_sutime;
double THR_sutime;
double GPS_sutime;
// relative time between local Sensor(LST) time to GPS UTC TOW in milliseconds (add time to LST to get UTC TOW of Sensor Measurement)
double IMUtoGPS;
double NANOtoGPS;
double ADPtoGPS;
double THRtoGPS;

// LOG file names
string IMULOGfile;
string GPSLOGfile;
string NANOLOGfile;
string ADPLOGfile;
string THRLOGfile;
// matching incoming serial data to message header
string imustartmatch;
string gpsstartmatch;
string nanostartmatch;
string adpstartmatch;
string thrstartmatch;

// pointer to shared memory(semaphore)
void* shmem_ptr_IMU;
void* shmem_ptr_msg;
void* shmem_ptr_GPS;
void* shmem_ptr_ADP;
void* shmem_ptr_NANO;
void* shmem_ptr_THR;
sem_t* msg_sem;

// creates shared memory, which can be accesed from all processes that created sh.mem. with same name
void* create_shared_mem(const int size, const char* name);

// initialization of everything (GPS, Synchronisation, LOG files, sh.mem.)
bool init(void);

// sets up GPS receiver to disbale NMEA Messages, and receive NAV-PVT Messages to check valid fix in init process
bool gpsinit(void);

// sends GPS configs to the receiver
bool sendtoGPS(const unsigned char send[], int size);

// calculates checksum to ensure a valid received message and no packet loss
bool GpsChecksum(char* Datench, int size );

// checks if new Process Message Protocoll (PMP) Message is placed in shared memory
bool checknewMSGdata(void);
// parses received Message float
void MSGparse(void);


// parse incoming data from serial
bool imudata(void);
// parse incoming message from serial
bool imumessage(void);
// first instance to assure incoming serial data is aligned to begining of data or message
bool IMUparse(void);

// parse incoming data from serial
bool nanodata(void);
// parse incoming message from serial
bool nanomessage(void);
// first instance to assure incoming serial data is aligned to begining of data or message
bool NANOparse(void);

// parse incoming data from serial
bool adpudata(void);
// parse incoming message from serial
bool adpumessage(void);
// first instance to assure incoming serial data is aligned to begining of data or message
bool ADPparse(void);

// parse incoming data from serial
bool thrudata(void);
// parse incoming message from serial
bool thrumessage(void);
// first instance to assure incoming serial data is aligned to begining of data or message
bool THRparse(void);

// parse incoming Serial messages over UART GPIO from GPS receiver
bool GPSparse(void);

// creates new log names for any LOG, to ensure that no LOG file gets overwritten
string LOGname(const char* location, const char* prefix);

// LOG data to created LOG files in ~/instrumentation_logs/SENSOR directory
void LOGIMUdata(void);

void LOGGPSdata(void);

void LOGNANOdata(void);

void LOGADPdata(void);

void LOGTHRdata(void);

// all Arduino Microprocessors wait on start up in a while loop. When Sart Signal ("S") is received through Serial, the millis() are sent back.
// Alowing to relate the millis() with the raspberry system time when signal was sent. The timelag between sending Signal and receiving millis() is ~1ms
void syncIMU(void);

void syncNANO(void);

void syncADP(void);

void syncTHR(void);

// previous set NAV-PVT message is disabled and TIM-TM2 Message is enabled in order to send an interrupt signal through GPIO and get the exact
// GPS UTC TOW (nanosecond accuracy) when Signal was received. Alowing to relate System time to GPS UTC time
void syncGPS(void);

// relates the previus received Arduino start up times to GPS UTC time. Biggest uncertanity in this sync routine is the Raspberry system time, which is
// not exact, due to the lag of a RTOS.
void sync(void);

// releases shared memory and sends reset Signal to all Arduinos.
bool cleanup(void);

// Handles and write General LOG
void LOGinfo(int casenmbr);
