#include <iostream>
#include <semaphore.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/types.h>
#include <cstring>

using namespace std;

#define SH_MEM_NAME_MSG "MSGshmem"
#define MSG_SEM_NAME "/msgsem"

bool alloff = false;
void* shmem_ptr_msg;
sem_t* msg_sem;
float MSGfloat = 999;

void* create_shared_mem(const int size, const char* name);

bool init(void);
