#ifndef STRAPDOWN_H
#define STRAPDOWN_H

#include <iostream>
#include <fstream>
#include <serial/serial.h>
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
#include <spawn.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <math.h>
#include <sstream>
#include <semaphore.h>
#include <unistd.h>

#include <eigen3/Eigen/Eigen>
using namespace Eigen;
using namespace std;

#define IMU_BAUD 1000000
#define IMU_PORT "/dev/IMU"

#define SH_MEM_NAME_IMU "IMUshmem"
#define SH_MEM_NAME_MSG "MSGshmem"
#define SH_MEM_NAME_STRAP "STRAPshmem"
#define SH_MEM_NAME_GPS "GPSshmem"

#define MSG_SEM_NAME "/msgsem"

#define LOG_LOCATION "/home/pi/instrumentation_logs/STRAP/"
#define LOG_PREFIX "STRAPLOG_"
#define LOG_SUFFIX "txt"
#define MAX_LOG_FILE 999

#define INFO_LOG "/home/pi/instrumentation_logs/INFO_LOG.txt"

typedef struct IMUdata{
  unsigned long IMUtimestamp; // local raspberry timestamp
  unsigned long IMUtime;
  float variables[6];
};

typedef struct STRAPdata{
  unsigned long STRAPtimestamp; // local raspberry timestamp
  unsigned long IMUtime;
  unsigned long IMUtime_old;
  float roll;
  float pitch;
  float yaw;
  Vector3d Vneb;
  double lat;
  double lon;
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

void* shmem_ptr_strap;
void* shmem_ptr_IMU;
void* shmem_ptr_msg;
void* shmem_ptr_GPS;
sem_t* msg_sem;

string LOGfile;

bool processbool = true;
bool IMUcalib = false;
bool receivedYaw = false;
bool LOGbool = false;

float MSGfloat = 999;
float latteiler, lonteiler, Reteiler, Rnteiler;
double lat_new, lon_new, h_new;
float pitchStrap, rollStrap, yawStrap;
Vector3d gravy;
const float d2r = 0.0174533; // degrees to radians
const float r2d = (1.0/0.0174533) ; // radians to degree
Vector3d Vneb_old;
double dT;
Matrix3d Omega_bnb;
const float PI2 = M_PI * 2;
Matrix3d IMUmountingcorrection;
double offroll = 0;
double offpitch = 0.0349066;
double offyaw = 0;

// calcAtt -- Strapdown calculation of Attiude with 1st order simplification
void calcAtt(void);

// calcVel -- Strapdown calculation of Velocity with constant value for integration simplification
void calcVel(void);

// calcPos -- Strapdown calulation of Position
void calcPos(void);

void calcStrap(void);

inline float fastsin(float x);

inline float fastcos(float x);

void* create_shared_mem(const int size, const char* name);

bool checknewdata(void);

bool checknewMSGdata(void);

void MSGparse(void);

void sendtoIMU(string message);

string LOGname(void);

void LOGdata(void);

void initStrap(void);

bool init(void);

bool cleanup(void);

void LOGinfo(int casenmbr);

#endif // STRAPDOWN_H
