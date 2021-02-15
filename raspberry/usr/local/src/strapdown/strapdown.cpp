#include <strapdown.h>

IMUdata IMU;
STRAPdata STRAP;
GPSdata GPS;

serial::Serial IMUser(IMU_PORT,IMU_BAUD);

int main(int argc, char const *argv[]) {
  init();
  while (processbool) {
    if (checknewMSGdata()) {
      MSGparse();
    }
    if (checknewdata()) {
      memcpy(&IMU, shmem_ptr_IMU, sizeof(IMU));
      calcStrap();
      memcpy(shmem_ptr_strap, &STRAP, sizeof(STRAP));
      if (LOGbool) {
        LOGdata();
      }
    }
  }
  return 0;
}

void calcAtt(void){
  Vector3d wbnb;
  Matrix3d Cnb_old;
  float a,b,c,d,a2,b2,c2,d2;
  Vector3d dsigma;
  float betragsigma, betragsigmainv;
  Vector4d quat_old, rk;
  Matrix4d mquat_old;

  STRAP.IMUtime_old = STRAP.IMUtime;
  STRAP.IMUtime = IMU.IMUtime;
	dT = (double)(STRAP.IMUtime - STRAP.IMUtime_old) * 0.001;
	Cnb_old = STRAP.Cnb_new;
	mquat_old << STRAP.quat_new.coeff(0),-STRAP.quat_new.coeff(1),-STRAP.quat_new.coeff(2),-STRAP.quat_new.coeff(3),
							 STRAP.quat_new.coeff(1),STRAP.quat_new.coeff(0),-STRAP.quat_new.coeff(3),STRAP.quat_new.coeff(2),
							 STRAP.quat_new.coeff(2),STRAP.quat_new.coeff(3),STRAP.quat_new.coeff(0),-STRAP.quat_new.coeff(1),
							 STRAP.quat_new.coeff(3),-STRAP.quat_new.coeff(2),STRAP.quat_new.coeff(1),STRAP.quat_new.coeff(0);

	wbnb = Vector3d(IMU.variables[3] - STRAP.bwx, IMU.variables[4] - STRAP.bwy, IMU.variables[5] - STRAP.bwz) * d2r - Cnb_old * ( STRAP.w_nen + STRAP.w_nie );
	dsigma = wbnb * dT;
	Omega_bnb << 0,-dsigma.coeff(2),dsigma.coeff(1),
							 dsigma.coeff(2),0,-dsigma.coeff(0),
							 -dsigma.coeff(1),dsigma.coeff(0),0;
	betragsigma = dsigma.norm();
	betragsigmainv = 1/betragsigma;

	rk =  Vector4d(fastcos(betragsigma * 0.5f), dsigma.coeff(0) * betragsigmainv * fastsin(betragsigma * 0.5f), dsigma.coeff(1) * betragsigmainv * fastsin(betragsigma * 0.5f), dsigma.coeff(2) * betragsigmainv * fastsin(betragsigma * 0.5f));

	STRAP.quat_new = mquat_old * rk;
	a = STRAP.quat_new.coeff(0);
	b = STRAP.quat_new.coeff(1);
	c = STRAP.quat_new.coeff(2);
	d = STRAP.quat_new.coeff(3);
	a2 = a * a;
	b2 = b * b;
	c2 = c * c;
	d2 = d * d;
	STRAP.Cnb_new << a2+b2-c2-d2, 2*(b*c+a*d), 2*(b*d-a*c),
						       2*(b*c-a*d), a2-b2+c2-d2, 2*(c*d+a*b),
						       2*(b*d+a*c), 2*(c*d-a*b), a2-b2-c2+d2;

	rollStrap = atan2(STRAP.Cnb_new.coeff(1,2), STRAP.Cnb_new.coeff(2,2));
	pitchStrap = -asin(STRAP.Cnb_new.coeff(0,2));
	yawStrap = atan2(STRAP.Cnb_new.coeff(0,1), STRAP.Cnb_new.coeff(0,0));

	STRAP.roll = rollStrap * r2d;
	STRAP.pitch = -pitchStrap * r2d;
  yawStrap = yawStrap * r2d;
  if (yawStrap < 0) yawStrap = 360.0 + yawStrap;
	STRAP.yaw = yawStrap;
}

void calcVel(void){
  Vector3d V_acc, V_cor, acc;

	Vneb_old = STRAP.Vneb;
	acc = Vector3d(IMU.variables[0] - STRAP.bax, IMU.variables[1] - STRAP.bay, IMU.variables[2] - STRAP.baz) * 9.80665;
	V_acc = STRAP.Cnb_new.transpose() * dT * (acc);// + 0.5f * Omega_bnb * acc);
	V_cor = (gravy - ( 2.0f * STRAP.Omega_nie + STRAP.Omega_nen) * Vneb_old) * dT;

	STRAP.Vneb = Vneb_old + V_acc + V_cor;
}

void calcPos(void){
  float h_old;
  double lat_old, lon_old;
	h_old = STRAP.h;
	lat_old = STRAP.lat * d2r;
	lon_old = STRAP.lon * d2r;

	h_new = h_old + dT * 0.5f * (STRAP.Vneb.coeff(2) + Vneb_old.coeff(2));
  lat_new = lat_old + dT * 0.5f * latteiler * (Vneb_old.coeff(0) + STRAP.Vneb.coeff(0));
	lon_new = lon_old + dT * 0.5f * lonteiler * (Vneb_old.coeff(1) + STRAP.Vneb.coeff(1));

	STRAP.w_nen = Vector3d(STRAP.Vneb.coeff(1) * Reteiler, -STRAP.Vneb.coeff(0) * Rnteiler, -STRAP.Vneb.coeff(1)*tan(lat_new)) * Reteiler;
	STRAP.Omega_nen << 0, -STRAP.w_nen.coeff(2), STRAP.w_nen.coeff(1),
                     STRAP.w_nen.coeff(2), 0, -STRAP.w_nen.coeff(0),
                     -STRAP.w_nen.coeff(1), STRAP.w_nen.coeff(0), 0;

	STRAP.h = h_new;
	STRAP.lat = lat_new * r2d;
	STRAP.lon = lon_new * r2d;
}

// schnelle Sinus Implementierung
inline float fastsin(float x){
    if(M_PI < x)
    {
        x = x-static_cast<int>((x+M_PI)/(PI2))*PI2;
    }
    else if(x < -M_PI)
    {
        x = x-static_cast<int>((x-M_PI)/(PI2))*PI2;
    }

    return x*(1 - x*x*(0.16666667f - x*x*(0.00833333f - x*x*(0.0001984f - x*x*0.0000027f))));
}
// schnelle Kosinus Implementierung
inline float fastcos(float x){
    if(M_PI < x)
    {
        x = x-static_cast<int>((x+M_PI)/(PI2))*PI2;
    }
    else if(x < -M_PI)
    {
        x = x-static_cast<int>((x-M_PI)/(PI2))*PI2;
    }

    return 1-x*x*(0.5f-x*x*(0.04166667f-x*x*(0.00138889f-x*x*(0.00002480f-x*x*0.000000275f))));
}

void calcStrap(void) {
  calcAtt();
  calcVel();
  calcPos();
}

void* create_shared_mem(const int size, const char* name){
  int shmem_fd;
  shmem_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
  ftruncate(shmem_fd, size);
  return mmap(NULL, size, PROT_READ|PROT_WRITE,  MAP_SHARED, shmem_fd, 0);
}

bool checknewdata(void){
  unsigned long check = *(unsigned long*)shmem_ptr_IMU;
  if (IMU.IMUtime != check) {
    return true;
  }
  return false;
}

bool checknewMSGdata(void) {
  float check = *(float*)shmem_ptr_msg;
  if (MSGfloat != check && (((int)check % 10 == 2) | ((int)check % 10 == 0))) {
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
    case 888: MSGfloat = 999000;
              memcpy(shmem_ptr_msg, &MSGfloat, sizeof(MSGfloat));
              IMUcalib = true;
      break;
    case 444: LOGinfo(6);
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
                LOGinfo(7);
                processbool = false;
              }
      break;
  }
  sem_post(msg_sem);
}

void sendtoIMU(const string message){
  IMUser.write(message);
}

string LOGname(void){
  int index = 0;
  int filepresent;
  string name;
  for (size_t i = 0; i <= MAX_LOG_FILE; i++) {
    name = LOG_LOCATION;
    name += LOG_PREFIX;
    name += to_string(index);
    name += ".";
    name += LOG_SUFFIX;
    const char * namech = name.c_str();
    filepresent = access(namech, F_OK);
    if (filepresent != 0) {
      mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
      open(namech, O_CREAT, mode);
      return name;
    }
    index++;
  }
  return 0;
}

void LOGdata(void){
  // char LOGbuffer[200];
  // unsigned int LOGbuffer_length;
  // int file;
  //
  // file = open(LOGfile.c_str(), O_WRONLY | O_APPEND );
  // if (!file) {
  //   LOGinfo(2);
  // }
  // LOGbuffer_length = sprintf(LOGbuffer,"%lu;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.8f;%.8f;%.2f\n", STRAP.IMUtime, STRAP.roll, STRAP.pitch, STRAP.yaw, STRAP.Vneb.coeff(0), STRAP.Vneb.coeff(1), STRAP.Vneb.coeff(2), STRAP.lat, STRAP.lon, STRAP.h);
  // char LOGstring[LOGbuffer_length];
  // strncpy(LOGstring, LOGbuffer, LOGbuffer_length);
  // write(file, LOGstring, sizeof(LOGstring));
  // fsync(file);
  // close(file);

  ofstream file;
  file.open(LOGfile, ios::out | ios::app);
  if (!file) {
    LOGinfo(2);
  }
  file << STRAP.IMUtime << ";";
  file << fixed << setprecision(2) << STRAP.roll << ";";
  file << fixed << setprecision(2) << STRAP.pitch << ";";
  file << fixed << setprecision(2) << STRAP.yaw << ";";
  file << fixed << setprecision(2) << STRAP.Vneb.coeff(0) << ";";
  file << fixed << setprecision(2) << STRAP.Vneb.coeff(1) << ";";
  file << fixed << setprecision(2) << STRAP.Vneb.coeff(2) << ";";
  file << fixed << setprecision(8) << STRAP.lat << ";";
  file << fixed << setprecision(8) << STRAP.lon << ";";
  file << fixed << setprecision(2) << STRAP.h << ";";
  file << endl;
  file.close();
}

void initStrap(void){
  Vector3d accel_ave(0, 0, 0);
  float Omega = 7.292e-5;
  IMUmountingcorrection << cos(offpitch)*cos(offyaw),-cos(offroll)*sin(offyaw)+sin(offroll)*sin(offpitch)*cos(offyaw),sin(offroll)*sin(offyaw)+cos(offroll)*sin(offpitch)*cos(offyaw),
                           cos(offpitch)*sin(offyaw),cos(offroll)*sin(offyaw)+sin(offroll)*sin(offpitch)*cos(offyaw),-sin(offroll)*sin(offyaw)+cos(offroll)*sin(offpitch)*cos(offyaw),
                           -sin(offpitch),sin(offroll)*cos(offpitch),cos(offroll)*cos(offpitch);

  memcpy(&GPS, shmem_ptr_GPS, sizeof(GPS));

  STRAP.lat = GPS.lat * 0.0000001;
  STRAP.lon = GPS.lon * 0.0000001;

  lat_new = STRAP.lat * ( M_PI/180 );
  lon_new = STRAP.lon * ( M_PI/180 );
  STRAP.w_nie = Vector3d(Omega*cos(lat_new),0 , -Omega*sin(lat_new));
  STRAP.Omega_nie << 0, -STRAP.w_nie.coeff(2), STRAP.w_nie.coeff(1),
                   STRAP.w_nie.coeff(2), 0, -STRAP.w_nie.coeff(0),
                   -STRAP.w_nie.coeff(1), STRAP.w_nie.coeff(0), 0;
  STRAP.Omega_nen << 0, 0, 0,
                     0, 0, 0,
                     0, 0, 0;

  double Halbachse_a = 6378137;
  double e2 = pow(0.0818191908426, 2);
  STRAP.R_n = Halbachse_a * ((1 - e2)/pow(1 - e2 * pow(sin(lat_new), 2), 3.0f/2.0f));
  STRAP.R_e = Halbachse_a/sqrt(1 - e2 * pow(sin(lat_new),2));

  latteiler = 1.0/(STRAP.R_n);
  lonteiler = 1.0/(STRAP.R_n * cos(lat_new));
  Reteiler = 1.0/(STRAP.R_e);
  Rnteiler = 1.0/(STRAP.R_n);

  STRAP.h = 0;
  STRAP.Vneb = Vector3d(0,0,0);
  gravy = Vector3d(0, 0, 9.80665);

  STRAP.bax = 0.001355;
  STRAP.bay = -0.00007;
  STRAP.baz = 0.014527659;

  STRAP.bwx = 0;
  STRAP.bwy = 0;
  STRAP.bwz = 0;

  sendtoIMU("s");
  int i = 0;
  for (i = 0; i < 1000; i++)
  {
    if ( !checknewdata() ){ // If no new data is available
      i -= 1;
      continue; // return to the top of the loop
    }
    memcpy(&IMU, shmem_ptr_IMU, sizeof(IMU));
    accel_ave += Vector3d(IMU.variables[0], IMU.variables[1], IMU.variables[2]);
  }
  accel_ave = accel_ave/1000;
  rollStrap = atan(accel_ave.coeff(1)/accel_ave.coeff(2));
  pitchStrap = asin(accel_ave.coeff(0));
  while (!receivedYaw) {
    sem_wait(msg_sem);
    memcpy(&MSGfloat, shmem_ptr_msg, sizeof MSGfloat);
    sem_post(msg_sem);
    if ((MSGfloat >= 0) && (MSGfloat <= 360)) {
      LOGinfo(3);
      sendtoIMU("i");
      receivedYaw = true;
    }
  }
  yawStrap = MSGfloat;
  MSGfloat = 9990000;
  sem_wait(msg_sem);
  memcpy(shmem_ptr_msg, &MSGfloat, sizeof(MSGfloat));
  sem_post(msg_sem);

  float a = cos(rollStrap*0.5f) * cos(pitchStrap*0.5f) * cos(yawStrap*0.5f) + sin(rollStrap*0.5f) * sin(pitchStrap*0.5f) * sin(yawStrap*0.5f);
  float b = sin(rollStrap*0.5f) * cos(pitchStrap*0.5f) * cos(yawStrap*0.5f) - cos(rollStrap*0.5f) * sin(pitchStrap*0.5f) * sin(yawStrap*0.5f);;
  float c = cos(rollStrap*0.5f) * sin(pitchStrap*0.5f) * cos(yawStrap*0.5f) + sin(rollStrap*0.5f) * cos(pitchStrap*0.5f) * sin(yawStrap*0.5f);;
  float d = cos(rollStrap*0.5f) * cos(pitchStrap*0.5f) * sin(yawStrap*0.5f) - sin(rollStrap*0.5f) * sin(pitchStrap*0.5f) * cos(yawStrap*0.5f);;
  STRAP.quat_new = Vector4d(a,b,c,d);
  STRAP.Cnb_new << pow(a, 2)+pow(b, 2)-pow(c, 2)-pow(d, 2), 2*(b*c+a*d), 2*(b*d-a*c),
                   2*(b*c-a*d), pow(a, 2)-pow(b, 2)+pow(c, 2)-pow(d, 2), 2*(c*d+a*b),
                   2*(b*d+a*c), 2*(c*d-a*b), pow(a, 2)-pow(b, 2)-pow(c, 2)+pow(d, 2);

  STRAP.roll = rollStrap * (180.0 / M_PI);
  STRAP.pitch = pitchStrap * (180.0 / M_PI);
	STRAP.yaw = yawStrap * (180.0 / M_PI);
}

bool init(void){
  bool startcalculations = false;

  shmem_ptr_IMU = create_shared_mem(sizeof(IMU), SH_MEM_NAME_IMU);
  shmem_ptr_msg = create_shared_mem(sizeof(MSGfloat), SH_MEM_NAME_MSG);
  shmem_ptr_strap = create_shared_mem(sizeof(STRAP), SH_MEM_NAME_STRAP);
  shmem_ptr_GPS = create_shared_mem(sizeof(GPS), SH_MEM_NAME_GPS);

  msg_sem = (sem_t*)create_shared_mem(sizeof(sem_t), MSG_SEM_NAME);
  sem_init(msg_sem, 1, 1);

  LOGfile = LOGname();
  const char * LOGfilech = LOGfile.c_str();
  if (access(LOGfilech, F_OK) != 0) {
    LOGinfo(1);
    return false;
  }
  LOGinfo(5);

  while (!IMUcalib) {
    if (checknewMSGdata()) {
      MSGparse();
    }
  }

  initStrap();
  LOGinfo(4);
  while (!startcalculations) {
    int check;
    cout << "If yout want to start Logging and Calculations type in '777' : ";
    cin >> check;
    if (cin && check == 777) {
      cout << endl << "starting calculations" << endl;
      LOGinfo(8);
      MSGfloat = 777120;
      LOGbool = true;
      sem_wait(msg_sem);
      memcpy(shmem_ptr_msg, &MSGfloat, sizeof(MSGfloat));
      sem_post(msg_sem);
      startcalculations = true;
    }
  }
  memcpy(&IMU, shmem_ptr_IMU, sizeof(IMU));
  STRAP.IMUtime = IMU.IMUtime;
  return true;
}

bool cleanup(void){
  LOGbool = false;

  shm_unlink(SH_MEM_NAME_IMU);
  shm_unlink(SH_MEM_NAME_MSG);
  shm_unlink(SH_MEM_NAME_STRAP);
  sem_destroy(msg_sem);
  return true;
}

void LOGinfo(int casenmbr){
  ostringstream message;
  switch (casenmbr) {
    case 1: message << "not able to create LOG file";
      break;
    case 2: message << "not able to access LOG file";
            cout << "STRAPDOWN: not able to access LOG file" << endl;
      break;
    case 3: message << "Received Yaw initialization";
            cout << "Received Yaw initialization" << endl;
      break;
    case 4: message << "finished init()";
      break;
    case 5: message << "current LOGfile:" << LOGfile;
      break;
    case 6: message << "Received System shutdowncall";
      break;
    case 7: message << "cleaned everything up, now terminating process";
      break;
    case 8: message << "Received User Log Signal, starting calculations";
            cout << "STRAPDOWN: starting LOGs" << endl;
      break;
  }
  string msg = message.str();
  auto timenow = chrono::system_clock::to_time_t(chrono::system_clock::now());
  ofstream  info;
  info.open(INFO_LOG, ios::out | ios::app);
  info << "STRAPDOWN: " << ctime(&timenow) << "   current IMU time: " << STRAP.IMUtime << ", " << msg << endl;
}
