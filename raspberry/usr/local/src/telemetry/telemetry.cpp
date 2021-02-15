#include <telemetry.h>

GPSdata GPS;
STRAPdata STRAP;
ADPdata ADP;
NANOdata NANO;
THRUSTdata THR;
IMUdata IMU;

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	//init();

  shmem_ptr_strap = create_shared_mem(sizeof(STRAP), SH_MEM_NAME_STRAP);
  shmem_ptr_msg = create_shared_mem(sizeof(MSGfloat), SH_MEM_NAME_MSG);
  shmem_ptr_gps = create_shared_mem(sizeof(GPS), SH_MEM_NAME_GPS);
  shmem_ptr_adp = create_shared_mem(sizeof(ADP), SH_MEM_NAME_ADP);
  shmem_ptr_nano = create_shared_mem(sizeof(NANO), SH_MEM_NAME_NANO);
  shmem_ptr_thr = create_shared_mem(sizeof(THR), SH_MEM_NAME_THR);
  shmem_ptr_imu = create_shared_mem(sizeof(IMU), SH_MEM_NAME_IMU);

  msg_sem = (sem_t*)create_shared_mem(sizeof(sem_t), MSG_SEM_NAME);
  sem_init(msg_sem, 1, 1);

	string address = (argc > 1) ? string(argv[1]) : DFLT_ADDRESS;
	mqtt::async_client cli(address, "", MAX_BUFFERED_MSGS);
	mqtt::connect_options connOpts;
	// Create topic opjects
	mqtt::topic top(cli, TOPIC, QOS_data, true);

  mqtt::topic topic_gps_iTOW(cli, TOPIC_GPS_iTOW, QOS_data, true);
  mqtt::topic topic_gps_fixType(cli, TOPIC_GPS_fixType, QOS_data, true);
  mqtt::topic topic_gps_lon(cli, TOPIC_GPS_lon, QOS_data, true);
  mqtt::topic topic_gps_lat(cli, TOPIC_GPS_lat, QOS_data, true);
  mqtt::topic topic_gps_heightMS(cli, TOPIC_GPS_heightMS, QOS_data, true);
  mqtt::topic topic_gps_gSpeed(cli, TOPIC_GPS_gSpeed, QOS_data, true);
  mqtt::topic topic_gps_headingMotion(cli, TOPIC_GPS_headingMotion, QOS_data, true);
  mqtt::topic topic_gps_headingVehicle(cli, TOPIC_GPS_headingVehicle, QOS_data, true);

  mqtt::topic topic_nano_vdot(cli, TOPIC_NANO_vdot, QOS_data, true);
  mqtt::topic topic_nano_v(cli, TOPIC_NANO_v, QOS_data, true);
  mqtt::topic topic_nano_ch1(cli, TOPIC_NANO_ch1, QOS_data, true);
  mqtt::topic topic_nano_ch2(cli, TOPIC_NANO_ch2, QOS_data, true);
  mqtt::topic topic_nano_ch3(cli, TOPIC_NANO_ch3, QOS_data, true);
  mqtt::topic topic_nano_ch4(cli, TOPIC_NANO_ch4, QOS_data, true);
  mqtt::topic topic_nano_ch5(cli, TOPIC_NANO_ch5, QOS_data, true);
  mqtt::topic topic_nano_ch6(cli, TOPIC_NANO_ch6, QOS_data, true);
  mqtt::topic topic_nano_ch7(cli, TOPIC_NANO_ch7, QOS_data, true);
  mqtt::topic topic_nano_ch8(cli, TOPIC_NANO_ch8, QOS_data, true);
  mqtt::topic topic_nano_ch9(cli, TOPIC_NANO_ch9, QOS_data, true);
  mqtt::topic topic_nano_ch10(cli, TOPIC_NANO_ch10, QOS_data, true);
  mqtt::topic topic_nano_ch11(cli, TOPIC_NANO_ch11, QOS_data, true);
  mqtt::topic topic_nano_ch12(cli, TOPIC_NANO_ch12, QOS_data, true);
  mqtt::topic topic_nano_ch13(cli, TOPIC_NANO_ch13, QOS_data, true);

  mqtt::topic topic_strap_roll(cli, TOPIC_STRAP_roll, QOS_data, true);
  mqtt::topic topic_strap_pitch(cli, TOPIC_STRAP_pitch, QOS_data, true);
  mqtt::topic topic_strap_yaw(cli, TOPIC_STRAP_yaw, QOS_data, true);

  mqtt::topic topic_imu_AccX(cli, TOPIC_IMU_AccX, QOS_data, true);
  mqtt::topic topic_imu_AccY(cli, TOPIC_IMU_AccY, QOS_data, true);
  mqtt::topic topic_imu_AccZ(cli, TOPIC_IMU_AccZ, QOS_data, true);
  mqtt::topic topic_imu_GyroX(cli, TOPIC_IMU_GyroX, QOS_data, true);
  mqtt::topic topic_imu_GyroY(cli, TOPIC_IMU_GyroY, QOS_data, true);
  mqtt::topic topic_imu_GyroZ(cli, TOPIC_IMU_GyroZ, QOS_data, true);

  mqtt::topic topic_thr_force1(cli, TOPIC_THR_force1, QOS_data, true);
  mqtt::topic topic_thr_force2(cli, TOPIC_THR_force2, QOS_data, true);
  mqtt::topic topic_thr_temp1(cli, TOPIC_THR_temp1, QOS_data, true);
  mqtt::topic topic_thr_temp2(cli, TOPIC_THR_temp2, QOS_data, true);

  mqtt::topic topic_adp_pstat(cli, TOPIC_ADP_pstat, QOS_data, true);
  mqtt::topic topic_adp_pdyn(cli, TOPIC_ADP_pdyn, QOS_data, true);


	// Random number generator [0 - 100]
	random_device rnd;
    mt19937 gen(rnd());
    uniform_int_distribution<> dis(0, 100);
  LOGinfo(3);

	try {
		// Connect to the MQTT broker
		LOGinfo(4);
		cli.connect(connOpts)->wait();
    LOGinfo(5);

		char tmbuf[32];
		unsigned nsample = 0;

		// The time at which to reads the next sample, starting now
		auto tm = steady_clock::now();

		while (processbool) {

			if (checknewMSGdata()) {
				MSGparse();
			}
      if (checknewADPdata()) {
        memcpy(&ADP, shmem_ptr_adp, sizeof(ADP));
      }
      if (checknewGPSdata()) {
        memcpy(&GPS, shmem_ptr_gps, sizeof(GPS));
      }
      if (checknewNANOdata()) {
        memcpy(&NANO, shmem_ptr_nano, sizeof(NANO));
      }
      if (checknewSTRAPdata()) {
        memcpy(&STRAP, shmem_ptr_strap, sizeof(STRAP));
      }
      if (checknewIMUdata()) {
        memcpy(&IMU, shmem_ptr_imu, sizeof(IMU));
      }
      if (checknewTHRdata()) {
        memcpy(&THR, shmem_ptr_thr, sizeof(THR));
      }

			// Pace the samples to the desired rate
			this_thread::sleep_until(tm);
			// Get a timestamp and format as a string
			time_t t = system_clock::to_time_t(system_clock::now());
			strftime(tmbuf, sizeof(tmbuf), "%F %T", localtime(&t));

			// Simulate reading some data
			int x = dis(gen);

			// Create the payload as a text CSV string
			string payload = to_string(++nsample) + "," +
								tmbuf + "," + to_string(x);

			// Publish to the topic
			//top.publish(std::move(payload));

      topic_gps_iTOW.publish(std::move(to_string(GPS.iTOW)));
      topic_gps_fixType.publish(std::move(to_string(GPS.fixType)));
      topic_gps_lon.publish(std::move(to_string(GPS.lon)));
      topic_gps_lat.publish(std::move(to_string(GPS.lat)));
      topic_gps_heightMS.publish(std::move(to_string(GPS.hMSL)));
      topic_gps_gSpeed.publish(std::move(to_string(GPS.gSpeed)));
      topic_gps_headingMotion.publish(std::move(to_string(GPS.heading)));
      topic_gps_headingVehicle.publish(std::move(to_string(GPS.headVeh)));

      topic_nano_vdot.publish(std::move(to_string(NANO.vdot)));
      topic_nano_v.publish(std::move(to_string(NANO.vges)));
      topic_nano_ch1.publish(std::move(to_string(NANO.channelvalue[0])));
      topic_nano_ch2.publish(std::move(to_string(NANO.channelvalue[1])));
      topic_nano_ch3.publish(std::move(to_string(NANO.channelvalue[2])));
      topic_nano_ch4.publish(std::move(to_string(NANO.channelvalue[3])));
      topic_nano_ch5.publish(std::move(to_string(NANO.channelvalue[4])));
      topic_nano_ch6.publish(std::move(to_string(NANO.channelvalue[5])));
      topic_nano_ch7.publish(std::move(to_string(NANO.channelvalue[6])));
      topic_nano_ch8.publish(std::move(to_string(NANO.channelvalue[7])));
      topic_nano_ch9.publish(std::move(to_string(NANO.channelvalue[8])));
      topic_nano_ch10.publish(std::move(to_string(NANO.channelvalue[9])));
      topic_nano_ch11.publish(std::move(to_string(NANO.channelvalue[10])));
      topic_nano_ch12.publish(std::move(to_string(NANO.channelvalue[11])));
      topic_nano_ch13.publish(std::move(to_string(NANO.channelvalue[12])));
    
      topic_strap_roll.publish(std::move(to_string(STRAP.roll)));
      topic_strap_pitch.publish(std::move(to_string(STRAP.pitch)));
      topic_strap_yaw.publish(std::move(to_string(STRAP.yaw)));

      topic_imu_AccX.publish(std::move(to_string(IMU.variables[0])));
      topic_imu_AccY.publish(std::move(to_string(IMU.variables[1])));
      topic_imu_AccZ.publish(std::move(to_string(IMU.variables[2])));
      topic_imu_GyroX.publish(std::move(to_string(IMU.variables[3])));
      topic_imu_GyroY.publish(std::move(to_string(IMU.variables[4])));
      topic_imu_GyroZ.publish(std::move(to_string(IMU.variables[5])));

      topic_thr_force1.publish(std::move(to_string(THR.force1)));
      topic_thr_force2.publish(std::move(to_string(THR.force2)));
      topic_thr_temp1.publish(std::move(to_string(THR.temp1)));
      topic_thr_temp2.publish(std::move(to_string(THR.temp2)));

      topic_adp_pstat.publish(std::move(to_string(ADP.P_static)));
      topic_adp_pdyn.publish(std::move(to_string(ADP.P_dyn)));

			tm += PERIOD;
		}

		// Disconnect
		LOGinfo(6);
		cli.disconnect()->wait();
		LOGinfo(7);
	}
	catch (const mqtt::exception& exc) {
		cerr << exc.what() << endl;
		return 1;
	}

 	return 0;
}

void* create_shared_mem(const int size, const char* name){
  int shmem_fd;
  shmem_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
  ftruncate(shmem_fd, size);
  return mmap(NULL, size, PROT_READ|PROT_WRITE,  MAP_SHARED, shmem_fd, 0);
}

bool checknewMSGdata(void) {
  float check = *(float*)shmem_ptr_msg;
  if (MSGfloat != check && (((int)check % 10 == 4) | ((int)check % 10 == 0))) {
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

bool checknewSTRAPdata(void) {
  unsigned long check = *(unsigned long*)shmem_ptr_strap;
  if (STRAP.IMUtime != check) {
    return true;
  }
  return false;
}

bool checknewIMUdata(void) {
  unsigned long check = *(unsigned long*)shmem_ptr_imu;
  if (IMU.IMUtime != check) {
    return true;
  }
  return false;
}

bool checknewADPdata(void) {
  unsigned long check = *(unsigned long*)shmem_ptr_adp;
  if (ADP.ADPtime != check) {
    return true;
  }
  return false;
}

bool checknewNANOdata(void) {
  unsigned long check = *(unsigned long*)shmem_ptr_nano;
  if (NANO.NANOtime != check) {
    return true;
  }
  return false;
}

bool checknewTHRdata(void) {
  unsigned long check = *(unsigned long*)shmem_ptr_thr;
  if (THR.THRtime != check) {
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
              LOGinfo(1);
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
    case 444: LOGinfo(2);
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
                LOGinfo(2);
                processbool = false;
              };
      break;
    default:
      break;
  }
  sem_post(msg_sem);
}

bool cleanup(void){
  LOGbool = false;

  shm_unlink(SH_MEM_NAME_STRAP);
  shm_unlink(SH_MEM_NAME_MSG);
  shm_unlink(SH_MEM_NAME_GPS);
  shm_unlink(SH_MEM_NAME_ADP);
  shm_unlink(SH_MEM_NAME_NANO);
  shm_unlink(SH_MEM_NAME_THR);
  shm_unlink(SH_MEM_NAME_IMU);
  sem_destroy(msg_sem);
  return true;
}

/*bool init(void){
	shmem_ptr_strap = create_shared_mem(sizeof(STRAP), SH_MEM_NAME_STRAP);
  shmem_ptr_msg = create_shared_mem(sizeof(MSGfloat), SH_MEM_NAME_MSG);
  shmem_ptr_gps = create_shared_mem(sizeof(GPS), SH_MEM_NAME_GPS);

  msg_sem = (sem_t*)create_shared_mem(sizeof(sem_t), MSG_SEM_NAME);
  sem_init(msg_sem, 1, 1);

	string address = (argc > 1) ? string(argv[1]) : DFLT_ADDRESS;
	mqtt::async_client cli(address, "", MAX_BUFFERED_MSGS);
	mqtt::connect_options connOpts;
	// Create a topic object. This is a conventience since we will
	// repeatedly publish messages with the same parameters.
	mqtt::topic top(cli, TOPIC, QOS_data, true);

	// Random number generator [0 - 100]
	random_device rnd;
    mt19937 gen(rnd());
    uniform_int_distribution<> dis(0, 100);
  LOGinfo(3);

	return true;
}*/

void LOGinfo(int casenmbr){
  ostringstream message;
  switch (casenmbr) {
		case 1: message << "Received LOG signal";
			break;
		case 2: message << "Received System shutdowncall";
			break;
    case 3: message << "Init finished";
			break;
    case 4: message << "Telemetry connecting to server...";
			break;
    case 5: message << "Connected";
			break;
    case 6: message << "Disconnecting from server...";
			break;
    case 7: message << "Disconnected";
			break;
    case 8: message << "got time";
			break;
    case 9: message << "generated data";
			break;
    case 10: message << "sleeping";
			break;
  }
  string msg = message.str();
  auto timenow = chrono::system_clock::to_time_t(chrono::high_resolution_clock::now());
  ofstream  info;
  info.open(INFO_LOG, ios::out | ios::app);
  info << "TELEMETRY: " << ctime(&timenow) << "   GPS iTOW: "<< GPS.iTOW << ", " << msg << endl;
}
