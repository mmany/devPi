#include <ecuhandler.h>

/* TO DO:
    - Finish implementing ecudata()
    - Finish implementing init()
    - check header file
    - implement MSGparse to start logging, and to end the serial communication
    - test it
    - set telemetry, and topics for ECU data

*/

// local process storage
ECUdata ECU;

serial::Serial ECUser(ECU_PORT,ECU_BAUD);


int main(int argc, char const *argv[]) {
  if (!init()) {
    LOGinfo(1);
    return 0;
  }
  while (processbool) {
    if (checknewMSGdata()) {
      MSGparse();
    }
    if (ECUser.available() > 0) {
      ECUparse();
    }

  }
  return 0;
}

bool ECUparse(void){ // see IMU parse function in serialhandler.cpp for explanation

  while (ecustartmatch[0] != ECU_SLAVE_ADRESS) {
    ecustartmatch = "";
    ECUser.read(ecustartmatch, 1);
  }
  
}

void LOGinfo(int casenmbr){
    ostringstream message;
    switch casenmbr{
        case 1: message << "not able to init";
                cout << "ECUHANDLER: not able to init" << endl;
            break;
    }

}

bool ecudata(void){ // see IMU parse functions for explenation

  string Daten = "";
  bool endbool = false;

  if (nanostartmatch[0] == 'b') {
    nanostartmatch = "";
    endbool = false;


    timestamp = std::chrono::high_resolution_clock::now().count();
    Daten += to_string(timestamp); //adding arrival timestamp to the data

    while (!endbool) {
      Daten += NANOser.read(1);
      if (Daten[Daten.size()-1] == '\r') {
        char Datench[Daten.size() - 1];
        Daten.copy(Datench, Daten.size()-2, 0);
        Datench[Daten.size()-2] = '\0';
        Daten = "";

        // if ( (sizeof(Datench) / sizeof(Datench[0])) != NANO_DATA_PACKAGE_SIZE){
        //   LOGinfo(19);
        //   return false;
        // }
        memcpy(shmem_ptr_ECU, Datench, sizeof ECU); // copy char arry to shared memory space
        memcpy(&ECU, Datench, sizeof ECU);
        if (LOGbool) {
          LOGECUdata();
        }
        endbool = true;
      }
    }
    if (endbool == true) {
      return true;
    }
  }
  return false;
}

bool init(void){

  const char * LOGfilech;

  ECUser.flush();


  shmem_ptr_ECU = create_shared_mem(sizeof(ECU), SH_MEM_NAME_ECU);


  ECULOGfile = LOGname(ECU_LOG_LOCATION, ECU_LOG_PREFIX);
  LOGfilech = ECULOGfile.c_str();
  if (!access(LOGfilech, F_OK) == 0) {
    LOGinfo(11);
    return false;
  }

  return true;
}

void* create_shared_mem(const int size, const char* name){
  int shmem_fd;
  shmem_fd = shm_open(name, O_CREAT | O_RDWR, 0666); //create sh.mem. in read an write mode and 0666 user access
  ftruncate(shmem_fd, size); //truncate it to right size
  return mmap(NULL, size, PROT_READ|PROT_WRITE,  MAP_SHARED, shmem_fd, 0); // map shared memory to sh.mem. pointer
}
