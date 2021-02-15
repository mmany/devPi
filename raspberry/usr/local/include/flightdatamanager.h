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
#include <sstream>
#include <semaphore.h>


#include </usr/local/include/kalmanfilter/LLSonlineFPR-KalmanFilter.h>


#define INFO_LOG "/home/pi/instrumentation_logs/INFO_LOG.txt"

#define SERIALHANDLER "/usr/local/bin/serialhandler"
#define STRAPDOWN "/usr/local/bin/strapdown"
#define TELEMETRY "/usr/local/bin/telemetry"

#define SH_MEM_NAME_STRAP "STRAPshmem"
#define SH_MEM_NAME_MSG "MSGshmem"
#define SH_MEM_NAME_GPS "GPSshmem"

#define MSG_SEM_NAME "/msgsem"

typedef struct STRAPdata{
  unsigned long STRAPtimestamp;
  unsigned long IMUtime;
  unsigned long IMUtime_old;
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

bool processbool = true;
bool LOGbool = false;
pid_t pid[3];
int pidint = 0;
void* shmem_ptr_strap;
void* shmem_ptr_msg;
void* shmem_ptr_gps; // pointer for gps shared memory data
sem_t* msg_sem;
extern char **environ;

std::string errorcode = "";

float MSGfloat = 999000;

void* create_shared_mem(const int size, const char* name);

bool spawnprocess(char* Process);

bool checknewMSGdata(void);

bool checknewSTRAPdata(void);

bool checknewGPSdata(void);

void MSGparse(void);

bool init(void);

bool cleanup(void);

void LOGinfo(int casenmbr);
