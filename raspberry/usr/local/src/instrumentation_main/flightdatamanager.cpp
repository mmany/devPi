#include <flightdatamanager.h>


STRAPdata STRAP;
GPSdata GPS;
LLS_KALMANFILTER KF;

using namespace std;
using namespace serial;

int main() {
  init();
  while (processbool) {

    if (checknewMSGdata()) {
      MSGparse();
    }
    if (checknewSTRAPdata()) {
      memcpy(&STRAP, shmem_ptr_strap, sizeof(STRAP));
    }
    if (checknewGPSdata()) {
      memcpy(&GPS, shmem_ptr_gps, sizeof(GPS));
    }
  }
  return 0;
}

void* create_shared_mem(const int size, const char* name){

  int shmem_fd;
  shmem_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
  ftruncate(shmem_fd, size);
  return mmap(NULL, size, PROT_READ|PROT_WRITE,  MAP_SHARED, shmem_fd, 0);
}

bool spawnprocess(char* Process){

  char *argv[] = {Process, (char *) 0};
  if (posix_spawn(&pid[pidint], Process, NULL, NULL, argv, environ) != 0) {
    LOGinfo(1);
    return false;
  }
  pidint += 1;
  return true;
}

bool checknewSTRAPdata(void) {

  unsigned long check = *(unsigned long*)shmem_ptr_strap;

  if (STRAP.IMUtime != check) {
    return true;
  }
  return false;
}

bool checknewMSGdata(void) {

  float check = *(float*)shmem_ptr_msg;

  if (MSGfloat != check && (((int)check % 10 == 3) | ((int)check % 10 == 0))) {
    return true;
  }
  return false;
}

bool checknewGPSdata(void) {
  unsigned long check = *(unsigned long*)shmem_ptr_gps;
  if (GPS.iTOW != check) {
    return true;
  }
  return false;
}

void MSGparse(void){
  int MSGadress;
  int MSGcase;

  sem_wait(msg_sem);
  memcpy(&MSGfloat, shmem_ptr_msg, sizeof(MSGfloat));


  MSGadress = (int)MSGfloat % 1000;
  MSGcase = (int)MSGfloat / 1000;

  switch (MSGcase) {
    case 999: MSGfloat = 999000;
      break;
    case 777: LOGbool = true; // received logging signal from Strapdowncalculation (Strapdown is initialized properly and user input confirmed logging start)
              LOGinfo(5);
              if (MSGadress == 320) {
                MSGfloat = 999000;
                memcpy(shmem_ptr_msg, &MSGfloat, sizeof(MSGfloat));
              }
              else{
                MSGadress += 100;
                MSGfloat = MSGcase * 1000 + MSGadress;
                memcpy(shmem_ptr_msg, &MSGfloat, sizeof(MSGfloat));
              }
      break;
    case 444: LOGinfo(3);
              if (MSGadress == 300) {
                MSGfloat = 999000;
                memcpy(shmem_ptr_msg, &MSGfloat, sizeof(MSGfloat));
              }
              else{
                MSGadress += 100;
                MSGfloat = MSGcase * 1000 + MSGadress;
                memcpy(shmem_ptr_msg, &MSGfloat, sizeof(MSGfloat));
              }
              if (cleanup()) {
                LOGinfo(4);
                processbool = false;
              };
      break;
    default:
      break;
  }
  sem_post(msg_sem);
}

bool init(){
  if (access(INFO_LOG, W_OK) != 0) {
    mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
    open(INFO_LOG, O_CREAT, mode);
    LOGinfo(2);
  }
  if (access(INFO_LOG, W_OK) != 0) {
    cout << "not able to acces INFO_LOG.txt" << endl << "Backup and Delete existing INFO_LOG.txt" << endl;
    processbool = false;
    return false;
  }
  LOGinfo(0);

  shmem_ptr_strap = create_shared_mem(sizeof(STRAP), SH_MEM_NAME_STRAP);
  shmem_ptr_msg = create_shared_mem(sizeof(MSGfloat), SH_MEM_NAME_MSG);
  shmem_ptr_gps = create_shared_mem(sizeof(GPS), SH_MEM_NAME_GPS);

  msg_sem = (sem_t*)create_shared_mem(sizeof(sem_t), MSG_SEM_NAME);
  sem_init(msg_sem, 1, 1);

  MSGfloat = 999000;
  sem_wait(msg_sem);
  memcpy(shmem_ptr_msg, &MSGfloat, sizeof MSGfloat);
  sem_post(msg_sem);

  if (!spawnprocess((char*)SERIALHANDLER)) {
    errorcode = "Serialhandler";
    return false;
  }
  if (!spawnprocess((char*)STRAPDOWN)) {
    errorcode = "Strapdown";
    return false;
  }
  if (!spawnprocess((char*)TELEMETRY)) {
    errorcode = "Telemetry";
    return false;
  }
  return true;
}

bool cleanup(void){
  LOGbool = false;

  shm_unlink(SH_MEM_NAME_STRAP);
  shm_unlink(SH_MEM_NAME_MSG);
  shm_unlink(SH_MEM_NAME_GPS);
  sem_destroy(msg_sem);
  return true;
}

void LOGinfo(int casenmbr){
  ostringstream message;
  switch (casenmbr) {
    case 0: message << "STARTING";
      break;
    case 1: message << "   unable to spawn process: " + errorcode;
            cout << "unable to spawn process: " + errorcode << endl;
      break;
    case 2: message << "   created general LOG file";
      break;
    case 3: message << "   Received System shutdowncall";
      break;
    case 4: message << "   cleaned everything up, now terminating process";
      break;
    case 5: message << "   Received LOG signal";
            cout << "DATA MANAGER: starting LOGs" << endl;
      break;
  }
  string msg = message.str();
  auto timenow = chrono::system_clock::to_time_t(chrono::system_clock::now());
  ofstream  info;
  info.open(INFO_LOG, ios::out | ios::app);
  if (casenmbr != 0 && casenmbr != 2) {
    info << "DATA MANAGER: " << ctime(&timenow) << msg << endl;
  }
  else info << endl << "Data Manager: " << ctime(&timenow) << msg << endl;
}
