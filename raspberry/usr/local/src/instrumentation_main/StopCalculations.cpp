#include <stopcalculations.h>

int main(int argc, char const *argv[]) {
  if (!init()) {
    return 0;
  }
  sem_wait(msg_sem);
  memcpy(shmem_ptr_msg, &MSGfloat, sizeof(MSGfloat));
  sem_post(msg_sem);

  while (!alloff) {
    sem_wait(msg_sem);
    memcpy(&MSGfloat, shmem_ptr_msg, sizeof(MSGfloat));
    sem_post(msg_sem);
    if ((int)MSGfloat == 999000) {
      cout << "all systems off and cleaned up propperly" << endl;
      sem_destroy(msg_sem);
      alloff = true;
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

bool init(void) {
  shmem_ptr_msg = create_shared_mem(sizeof(MSGfloat), SH_MEM_NAME_MSG);

  msg_sem = (sem_t*)create_shared_mem(sizeof(sem_t), MSG_SEM_NAME);
  sem_init(msg_sem, 1, 1);

  int check;
  cout << "If yout really want to quit Logging and Calculations type in '444' : ";
  cin >> check;
  if (cin && check == 444) {
    cout << endl << "shuting down the logging System" << endl;
    MSGfloat = 444000;
    return true;
  }
  else{
    return false;
  }
  return false;
}
