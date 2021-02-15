#include <random>
#include <string>
#include <thread>
#include <chrono>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include "mqtt/async_client.h"

#include <fstream>
#include <serial/serial.h>
#include <stdint.h>
#include <iomanip>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <spawn.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sstream>
#include <semaphore.h>
#include <wiringPi.h>

#include <eigen3/Eigen/Eigen>
using namespace Eigen;
using namespace std;
using namespace std::chrono;

#define INFO_LOG "/home/pi/instrumentation_logs/INFO_LOG.txt"

#define SH_MEM_NAME_STRAP "STRAPshmem"
#define SH_MEM_NAME_MSG "MSGshmem"
#define SH_MEM_NAME_GPS "GPSshmem"
#define SH_MEM_NAME_ADP "ADPshmem"
#define SH_MEM_NAME_NANO "NANOshmem"
#define SH_MEM_NAME_THR "THRshmem"
#define SH_MEM_NAME_IMU "IMUshmem"

#define MSG_SEM_NAME "/msgsem"

const std::string DFLT_ADDRESS { "tcp://localhost:1883" };

// Data Topics
const string TOPIC { "data/rand" };

const string TOPIC_GPS_iTOW { "data/gps/iTOW" };
const string TOPIC_GPS_fixType { "data/gps/fixType" };
const string TOPIC_GPS_lon { "data/gps/lon" };
const string TOPIC_GPS_lat { "data/gps/lat" };
const string TOPIC_GPS_heightMS { "data/gps/heightMS" };
const string TOPIC_GPS_gSpeed { "data/gps/gSpeed" };
const string TOPIC_GPS_headingMotion { "data/gps/headingMotion" };
const string TOPIC_GPS_headingVehicle { "data/gps/headingVehicle" };

const string TOPIC_NANO_vdot { "data/nano/vdot" };
const string TOPIC_NANO_v { "data/nano/v" };
const string TOPIC_NANO_ch1 { "data/nano/ch1" };
const string TOPIC_NANO_ch2 { "data/nano/ch2" };
const string TOPIC_NANO_ch3 { "data/nano/ch3" };
const string TOPIC_NANO_ch4 { "data/nano/ch4" };
const string TOPIC_NANO_ch5 { "data/nano/ch5" };
const string TOPIC_NANO_ch6 { "data/nano/ch6" };
const string TOPIC_NANO_ch7 { "data/nano/ch7" };
const string TOPIC_NANO_ch8 { "data/nano/ch8" };
const string TOPIC_NANO_ch9 { "data/nano/ch9" };
const string TOPIC_NANO_ch10 { "data/nano/ch10" };
const string TOPIC_NANO_ch11 { "data/nano/ch11" };
const string TOPIC_NANO_ch12 { "data/nano/ch12" };
const string TOPIC_NANO_ch13 { "data/nano/ch13" };

const string TOPIC_STRAP_roll { "data/strap/roll" };
const string TOPIC_STRAP_pitch { "data/strap/pitch" };
const string TOPIC_STRAP_yaw { "data/strap/yaw" };

const string TOPIC_IMU_AccX { "data/imu/AccX" };
const string TOPIC_IMU_AccY { "data/imu/AccY" };
const string TOPIC_IMU_AccZ { "data/imu/AccZ" };
const string TOPIC_IMU_GyroX { "data/imu/GyroX" };
const string TOPIC_IMU_GyroY { "data/imu/GyroY" };
const string TOPIC_IMU_GyroZ { "data/imu/GyroZ" };

const string TOPIC_THR_force1 { "data/thr/force1" };
const string TOPIC_THR_force2 { "data/thr/force2" };
const string TOPIC_THR_temp1 { "data/thr/temp1" };
const string TOPIC_THR_temp2 { "data/thr/temp2" };

const string TOPIC_ADP_pstat { "data/adp/pstat" };
const string TOPIC_ADP_pdyn { "data/adp/pdyn" };

const int QOS_data = 0;

const auto PERIOD = milliseconds(100);

const int MAX_BUFFERED_MSGS = 0;	// 120 * 5sec => 10min off-line buffering

const string PERSIST_DIR { "data-persist" };

typedef struct IMUdata{
  unsigned long IMUtime;
  unsigned long IMUtimestamp; // arrival timestamp
  float variables[6];
};

typedef struct STRAPdata{
  unsigned long IMUtime;
  unsigned long IMUtime_old;
  unsigned long STRAPtimestamp; // added timestamp on arrival
  float roll;
  float pitch;
  float yaw;
  Vector3d Vneb;
  float lat;
  float lon;
  float h;
  float R_n;
  float R_e;
  Vector4d quat_new;
  Matrix3d Cnb_new;
  Vector3d w_nie, w_nen;
  Matrix3d Omega_nie;
  Matrix3d Omega_nen;
  float bax;
  float bay;
  float baz;
  float bwx;
  float bwy;
  float bwz;
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

#define NANO_PPM_CHANNELS 13
typedef struct NANOdata{
  unsigned long NANOtime;
  unsigned long NANOtimestamp; // added local timestamp
  float vdot;
  float vges;
  short int channelvalue[NANO_PPM_CHANNELS];
};

typedef struct ADPdata{
  unsigned long ADPtime;
  unsigned long ADPtimestamp; // Added local timestamp
  float P_dyn;
  float P_static;
};

typedef struct THRUSTdata{
  unsigned long THRtime;
  unsigned long THRtimestamp; // Added local timestamp
  float temp1;
  float temp2;
  float force1;
  float force2;
  unsigned long raw1;
  unsigned long raw2;
  unsigned long THRdata;
};

bool processbool = true;
bool LOGbool = false;
void* shmem_ptr_strap;
void* shmem_ptr_msg;
void* shmem_ptr_gps; // pointer for gps shared memory data
void* shmem_ptr_adp;
void* shmem_ptr_nano;
void* shmem_ptr_thr;
void* shmem_ptr_imu;
sem_t* msg_sem;

float MSGfloat = 999000;

void* create_shared_mem(const int size, const char* name);

// checks if new Process Message Protocoll (PMP) Message is placed in shared memory
bool checknewMSGdata(void);
// parses received Message float
void MSGparse(void);

bool checknewSTRAPdata(void);

bool checknewGPSdata(void);

bool checknewADPdata(void);

bool checknewNANOdata(void);

bool checknewTHRdata(void);

bool checknewIMUdata(void);

bool init(void);

bool cleanup(void);

void LOGinfo(int casenmbr);
