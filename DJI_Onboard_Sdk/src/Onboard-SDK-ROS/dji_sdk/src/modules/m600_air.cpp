#include<serial/serial.h>
#include<iostream>
#include<cstring>
#include<stddef.h>
#include<string.h>
#include<ros/ros.h>
#include <stdio.h>
#include <exception>
#include<geometry_msgs/Twist.h>
//#include <dji_sdk/dji_sdk_node.h>
#include<dji_sdk/GimCtr.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/NavSatFix.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/Joy.h>
#include<pthread.h>
#include <iomanip>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include<tf/transform_datatypes.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Vector3Stamped.h>
#include<dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionWpAction.h>
#include<dji_sdk/MissionWaypoint.h>
#include<dji_sdk/MissionWaypointTask.h>
#include<dji_sdk/MissionWaypointAction.h>
#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionHpUpload.h>
#include <djiosdk/dji_vehicle.hpp>
#include<cmath>
#include<thread>
#include<boost/thread.hpp>
#include<iomanip>

//#include "dji_sdk/dji_sdk.h"

//#define MY_DEBUG 1

class M600_Air{

private:
	//align in one byte
	#pragma pack (1)
	struct HeartBeat{
		uint8_t sysPingHealth;
		uint8_t systemRetention[8];	
	};

	struct SystemStatus{
		int32_t battery;
		char lock_flag;
		uint8_t  authority;
		uint8_t  systemRetention[13];
	};

	struct GpsStatus{
		uint8_t positionSta;
		uint8_t effectiveSatelliteNum;
		int32_t latitude;
		int32_t longitude;
		uint16_t altitude;
		uint16_t groudVel;
		uint16_t gpsYaw;
		uint8_t hours;
		uint8_t minutes;
		uint8_t seconds;
		int16_t gpsXVel;
		int16_t gpsYVel;
		int16_t gpsZVel;	
	};
	
	struct ImuStatus{
		int32_t imuPitch;
		int32_t imuRoll;
		int32_t imuYaw;
		int32_t imuPitchVel;
		int32_t imuRollVel;
		int32_t imuYawVel;
		int32_t imuXAcss;
		int32_t imuYAcss;
		int32_t imuZAcss;
		int16_t imuXMSig;
		int16_t imuYMSig;
		int16_t imuZMSig;
	};
	
	struct OdomStatus{
		int32_t positionX;
		int32_t positionY;
		int32_t positionZ;
		int8_t velX;
		int8_t velY;
		int8_t velZ;
	};

	struct FlyCtrData{
		uint8_t emFlag;
		uint8_t mode;
		int16_t ref1;
		int16_t ref2;
		int16_t ref3;
		int16_t ref4;
		uint32_t latitude;
		uint32_t longitude;
	};
	
	struct GimCtrData{
		int16_t pitchCtr;
		int16_t yawCtr;
		int16_t setMutiple;
		int16_t setFocus;
		int8_t reset;
		int8_t camSOS;
		int8_t camSOR;
	};

	struct WaypointMission{
		uint8_t missionIndex;
		uint8_t latLntCount;
		int32_t ** latLntList;
		uint8_t actionCount;
		uint8_t ** waypointAction;
		uint16_t altitude;
		uint8_t velocity;
		uint8_t finishAction;
		uint8_t headingMode;
		int16_t gimbalAngle;
		int8_t repeatTimes;
		int8_t isUseCam;
		int8_t sor;
		int8_t shootVel;
	};

	struct HotpointMission{
		uint8_t missionIndex;
		int32_t latLnt[2];
		uint8_t startHotPoint;
		uint8_t headingMode;
		uint16_t altitude;
		uint8_t angleVel;
		uint32_t radius;
		int16_t gimbalAngel;
		uint8_t isUseCam;
		uint8_t sor;
		uint8_t shootVel;
	};

	struct MissionAction{
		uint8_t missionType;
		uint8_t actionType;
	};

	template <class T> 
	struct DataFrame{
		uint8_t frameHead[2];
		uint8_t  dataLen;
		uint8_t  frameIndex;
		uint8_t  systemNum;
		uint8_t  moduleNum;
		uint8_t  msgPkgNum;
		T*	 data ;
		uint8_t checkSum[2];
	};
	#pragma pack ()
	// tw
	enum CtrData{
		REV_FCTR,
		REV_GIMCTR,
		REV_SYS,
		REV_WPM,
		REV_HPM,
		REV_MAC
	};

	typedef struct ServiceAck{
		bool result;
		int cmd_set;
		int cmd_id;
		unsigned int ack_data;
		ServiceAck(bool res, int set, int id, unsigned int ack)
			: result(res)
			, cmd_set(set)
			, cmd_id(id)
			, ack_data(ack){}
			ServiceAck(){}
	} ServiceAck;

	// class private contribute
	DataFrame<SystemStatus>* systemStatusFrame = NULL;
	DataFrame<GpsStatus>*  gpsStatusFrame = NULL;
	DataFrame<ImuStatus>*  imuStatusFrame = NULL;
	DataFrame<OdomStatus>*  odomStatusFrame = NULL;		

	SystemStatus* revSystemStatusData =NULL;
	FlyCtrData* revFlyCtrData = NULL;
	GimCtrData* revGimCtrData = NULL;
	MissionAction* revMAction =NULL;
	HotpointMission * revHMission = NULL;
	
	

	serial::Serial* mySerial = NULL;

	ros::Subscriber imuSub;
	ros::Subscriber velSub;
	ros::Subscriber gpsSub;

	sensor_msgs::Imu imuData ;
	geometry_msgs::Vector3Stamped velData;
	sensor_msgs::NavSatFix gpsData;
	geometry_msgs::Vector3 mVel;

	ros::Publisher	fCtrPub;
	sensor_msgs::Joy joyVelCtr;

	ros::Publisher	gimCtrPub;
	dji_sdk::GimCtr gimCtr;

	ros::Rate send_data_rate = 5;

	pthread_t revLoopThread;
	pthread_t sendLoopThread;

	//service client
   	ros::ServiceClient sdk_ctrl_authority_service;
	ros::ServiceClient drone_task_service;

	//waypoint&&hotpoint
	dji_sdk::MissionWaypointTask waypointTask;
	dji_sdk::MissionWaypoint waypoint;
	dji_sdk::MissionWaypointAction wayPointAction;
	ros::ServiceClient  waypoint_upload_service;
	ros::ServiceClient  waypoint_action_service;


	dji_sdk::MissionHotpointTask hotpointTask;
	ros::ServiceClient  hotpoint_upload_service;
	ros::ServiceClient  hotpoint_action_service;

	//thread
	boost::mutex gimHDMutex;
	boost::mutex gimSrcMutex;
	bool isContinueEndSrc = true;
	bool isContinueCanSrc = false;
	
	bool isSingleEndSrc = true;
	bool isSingleCanSrc = false;
	
	bool isRecordEndSrc = true;
	bool isRecordCanSrc =  false;


	//set and send byte data to serial
	template <class T> 
	void setAndSendByteArray(DataFrame<T> * dataFrame, int dataSize){
		uint16_t checksum = 0x0000;
		uint8_t * dataBuffer = new uint8_t[255];
		memmove(dataBuffer,dataFrame,7);
		//std::cout<<dataBuffer<<std::endl;
		memmove(&dataBuffer[7],dataFrame->data,dataSize);
		for(int i = 0 ;i<dataSize+7;i++){
			checksum += dataBuffer[i];
		}
		memmove(&dataBuffer[dataSize+7],&checksum,2);
		mySerial->write(dataBuffer,dataSize+7+2);
		//std::cout<<dataBuffer<<std::endl;
		delete dataBuffer;
	}
	//get new vel data
	void velSubCallback(const geometry_msgs::Vector3Stamped& velMsg){
		this->velData = velMsg;
	}
	//get new gps data
	void gpsSubCallback(const sensor_msgs::NavSatFix& gpsMsg){
		this->gpsData = gpsMsg;
	}
	//get new imu data
	void imuSubCallback(const sensor_msgs::Imu& imuMsg){
		this->imuData = imuMsg;
	}
	// init Pub and Sub
	void initSubPub(ros::NodeHandle &nh){
		imuSub = nh.subscribe("dji_sdk/imu",10,&M600_Air::imuSubCallback,this);
		velSub = nh.subscribe("dji_sdk/velocity",10,&M600_Air::velSubCallback,this);
		gpsSub = nh.subscribe("dji_sdk/gps_position",10,&M600_Air::gpsSubCallback,this);

		fCtrPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate",10);
		gimCtrPub = nh.advertise<dji_sdk::GimCtr>("/m600_gimCtr",10);
		sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");

 		drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
		waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>("dji_sdk/mission_waypoint_upload");
		waypoint_action_service = nh.serviceClient<dji_sdk::MissionWpAction>("dji_sdk/mission_waypoint_action");
		hotpoint_upload_service = nh.serviceClient<dji_sdk::MissionHpUpload>("dji_sdk/mission_hotpoint_upload");
		hotpoint_action_service = nh.serviceClient<dji_sdk::MissionHpAction>("dji_sdk/mission_hotpoint_action");
	}
	//  UAV send imu gps odom data to ground in 5 HZ
	void sendUavDataLoop(){
		while(ros::ok()){
			sendImuByteData();
			sendGpsByteData();
			sendOdomByteData();
			send_data_rate.sleep();
		}	
	}


	void quatToEu(const geometry_msgs::Quaternion mQuat, geometry_msgs::Vector3 &mEu){
		tf::Quaternion quat;
		//tf::quaternionMsgToTF(mQuat,quat);
		tf::Matrix3x3 mat(tf::Quaternion(mQuat.x,mQuat.y,mQuat.z,mQuat.w));
		
		double roll,pitch,yaw;
		mat.getEulerYPR(yaw,pitch,roll);
		//tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
		mEu.x = pitch;
		mEu.y = roll;
		mEu.z = yaw;	
	}


	// send imu byte data to p900 port
	void sendImuByteData(){
		if(imuStatusFrame == NULL){
			imuStatusFrame = new DataFrame<ImuStatus>();
			imuStatusFrame->data = new ImuStatus();
			imuStatusFrame->frameHead[0] = 0xFE;
			imuStatusFrame->frameHead[1] = 0xEF;
			imuStatusFrame->dataLen = 51;
			imuStatusFrame->frameIndex = 1;
			imuStatusFrame->systemNum = 0x01;
			imuStatusFrame->moduleNum = 0x00;
			imuStatusFrame->msgPkgNum = 0x04;
		}else{
			imuStatusFrame->frameIndex+=1;
		}
		quatToEu(this->imuData.orientation,this->mVel);
		
		//std::cout<<this->mVel.x*pow(10,16)<<" "<<this->mVel.y*pow(10,16)<<" "<<this->mVel.z*pow(10,16)<<std::endl;
		imuStatusFrame->data->imuPitch =(int32_t) (this->mVel.x*pow(10,16));
		imuStatusFrame->data->imuRoll = (int32_t)(this->mVel.y*pow(10,16));
		imuStatusFrame->data->imuYaw = (int32_t)(this->mVel.z*pow(10,16));
		imuStatusFrame->data->imuPitchVel = (int32_t)(this->imuData.angular_velocity.x*pow(10,16));
		imuStatusFrame->data->imuRollVel = (int32_t)(this->imuData.angular_velocity.y*pow(10,16));
		imuStatusFrame->data->imuYawVel = (int32_t)(this->imuData.angular_velocity.z*pow(10,16));
		imuStatusFrame->data->imuXAcss = (int32_t)(this->imuData.linear_acceleration.x*pow(10,16));
		imuStatusFrame->data->imuYAcss = (int32_t)(this->imuData.linear_acceleration.y*pow(10,16));
		imuStatusFrame->data->imuZAcss = (int32_t)(this->imuData.linear_acceleration.z*pow(10,16));
		imuStatusFrame->data->imuXMSig = 0;
		imuStatusFrame->data->imuYMSig = 0;
		imuStatusFrame->data->imuZMSig = 0;
		setAndSendByteArray<ImuStatus>(imuStatusFrame,42);
	}
	// send gps byte data to p900 port
	void sendGpsByteData(){
		if(gpsStatusFrame == NULL){
			gpsStatusFrame = new DataFrame<GpsStatus>();
			gpsStatusFrame->data = new GpsStatus();
			gpsStatusFrame->frameHead[0] =0xFE;
			gpsStatusFrame->frameHead[1] = 0xEF;
			gpsStatusFrame->dataLen = 34;
			gpsStatusFrame->frameIndex = 1;
			gpsStatusFrame->systemNum = 0x01;
			gpsStatusFrame->moduleNum = 0x00;
			gpsStatusFrame->msgPkgNum = 0x02;
		}else{
			gpsStatusFrame->frameIndex+=1;
		}
		gpsStatusFrame->data->positionSta = 0;
		gpsStatusFrame->data->effectiveSatelliteNum = 0;
		gpsStatusFrame->data->latitude = (int32_t)(this->gpsData.latitude*pow(10,7));
		gpsStatusFrame->data->longitude =  (int32_t)(this->gpsData.longitude*pow(10,7));
		gpsStatusFrame->data->altitude = (uint16_t)(this->gpsData.altitude);
		gpsStatusFrame->data->groudVel = 0;
		gpsStatusFrame->data->gpsYaw = 0;
		gpsStatusFrame->data->hours = 0;
		gpsStatusFrame->data->minutes = 0;
		gpsStatusFrame->data->seconds = 0;
		gpsStatusFrame->data->gpsXVel = 0;
		gpsStatusFrame->data->gpsYVel = 0;
		gpsStatusFrame->data->gpsZVel = 0;
		setAndSendByteArray<GpsStatus>(gpsStatusFrame,25);	
	}

	// send odom byte data to p900 port
	void sendOdomByteData(){
		if(odomStatusFrame == NULL){
			odomStatusFrame = new DataFrame<OdomStatus>();
			odomStatusFrame->data = new OdomStatus();
			odomStatusFrame->frameHead[0] =0xFE;
			odomStatusFrame->frameHead[1] = 0xEF;
			odomStatusFrame->dataLen = 24;
			odomStatusFrame->frameIndex = 1;
			odomStatusFrame->systemNum = 0x01;
			odomStatusFrame->moduleNum = 0x00;
			odomStatusFrame->msgPkgNum = 0x06;
		}else{
			odomStatusFrame->frameIndex+=1;
		}
		odomStatusFrame->data->positionX = 0*100;
		odomStatusFrame->data->positionY = 0*100;
		odomStatusFrame->data->positionZ = 0*100;
		odomStatusFrame->data->velX =  (int8_t)(this->velData.vector.x*10);
		odomStatusFrame->data->velY = (int8_t)(this->velData.vector.y*10);
		odomStatusFrame->data->velZ = (int8_t)(this->velData.vector.z*10);
		setAndSendByteArray<OdomStatus>(odomStatusFrame,15);
	}

	static void* revByteDataLoopStatic(void* object){
		reinterpret_cast<M600_Air*>(object)->revByteDataLoop();
		return 0;
	}
	static void* sendUavDataLoopStatic(void* object){
		reinterpret_cast<M600_Air*>(object)->sendUavDataLoop();
		return 0;
	}
    ////等各个线程退出后，进程才结束，否则进程强制结束了，线程可能还没反应过来；
    //pthread_exit(NULL);

	void revByteDataLoopThread(){
		int resul_rev = pthread_create(&revLoopThread,NULL,&M600_Air::revByteDataLoopStatic,this);
		if(resul_rev != 0){
			std::cout<<"pthread_create error:error_code ="<<resul_rev<<std::endl;
		}
		int result_send = pthread_create(&sendLoopThread,NULL,&M600_Air::sendUavDataLoopStatic,this);
		if(result_send != 0){
			std::cout<<"pthread_create error:error_code ="<<result_send<<std::endl;
		}
	}

	// continue to receive byte data from p900 port 
	void revByteDataLoop(){
		
		int dataLen = 0;
		uint8_t * dataBuffer = new uint8_t[255];
		uint8_t * completeOneData = new uint8_t[255];
		
		while(ros::ok()){
			//std::cout<<"yyyyyyyyyy"<<std::endl;
			int len = mySerial->available();
			if(len > 2 || (dataLen !=0 && len>0)){
				mySerial->read(&dataBuffer[dataLen],len);
				std::cout<<"byte1";
				for(int i = 0;i<dataLen+len;i++){
					printf(" %x ",dataBuffer[i]);
				}
				std::cout<<std::endl;
				for(int i = 0; i< len+dataLen - 2; i++){
					if(dataBuffer[i] == 0xFE && dataBuffer[i+1] == 0xEF){
						if(dataBuffer[i+2] <= len+dataLen - i){
							
							memmove(&completeOneData[0],&dataBuffer[i],dataBuffer[i+2]);
							parseOneFrame(completeOneData,dataBuffer[i+2]);
							
						}else{
							memmove(&dataBuffer[0],&dataBuffer[i],len+dataLen - i);
							dataLen = len+dataLen - i;
							break;
						}				
					}
					if(i == len+dataLen-3){
						dataLen = 0;
					}	
				}
			
			}
		}
	
		delete dataBuffer;
		delete completeOneData;
		
	}


	// parse one complete byte data and convert it to a frameStruct
	void parseOneFrame(uint8_t * byteData, int len){
		#ifdef MY_DEBUG
		#endif My_DEBUG
		uint16_t checkSum = 0x0000;
		uint8_t	low = 0x00;
		uint8_t high = 0x00;
		
		if(byteData[0] == 0xFE && byteData[1] == 0xEF && byteData[2] ==len){
			for(int i =0 ;i<byteData[2]-2;i++){
				checkSum +=byteData[i];
			}
			low = checkSum;
			high = checkSum>>8;
			std::cout<<"whether mission come"<<std::endl;
			if(high == byteData[len - 1] && low == byteData[len - 2]){
				std::cout<<"good byteArray"<<std::endl;
				uint8_t *key_data = new uint8_t[255];
				memmove(&key_data[0],&byteData[7],byteData[2] - 9);
				byteDataToFrame(key_data,byteData[2] - 9,byteData[6]);
				delete key_data;
			}
		}
				
	}


	//convert byte data to frame struct 
	void byteDataToFrame(uint8_t * keyData,int len,uint8_t dataKind){
		if(dataKind == 0x01){	
			revSystemStatusData = (SystemStatus*)keyData;
			frameToPubTopic((void*)revSystemStatusData,REV_SYS);
		}else if(dataKind == 0x10){
			std::cout<<"byte2";
			for(int i = 0;i<len;i++){
				printf(" %x ",keyData[i]);
			}
			std::cout<<std::endl;
			revFlyCtrData = (FlyCtrData*)keyData;
			frameToPubTopic((void*)revFlyCtrData,REV_FCTR);

		}else if(dataKind == 0x11){
			std::cout<<"byte3";
			for(int i = 0;i<len;i++){
				printf(" %x ",keyData[i]);
			}
			std::cout<<std::endl;
			revGimCtrData = (GimCtrData*)keyData;
			frameToPubTopic((void*)revGimCtrData,REV_GIMCTR);
		}else if(dataKind == 0x20){
			byteDataToWPMFrame(keyData);
		}else if(dataKind == 0x21){
			revHMission = (HotpointMission*)keyData;
			frameToPubTopic((void *)revHMission,REV_HPM);
		}else if(dataKind == 0x22){
			revMAction = (MissionAction*)keyData;
			frameToPubTopic((void *)revMAction,REV_MAC);
		}else{
			std::cout<<"byteToFrame error"<<std::endl;
		}
	}

	// parse  a frame and publish a topic 

	void frameToPubTopic(void* frameData, CtrData revData){
		switch(revData){
			case M600_Air::REV_FCTR:
				pubFCtrTopic((FlyCtrData*)frameData);
				std::cout<<"sys"<<std::endl;
				break;	
			case M600_Air::REV_GIMCTR:
				pubGimCtrTopic((GimCtrData*)frameData);
				break;
			case M600_Air::REV_SYS:
				parseSystem((SystemStatus*)frameData);
				break;
			case M600_Air::REV_HPM:
				byteDataToHPMFrame((HotpointMission*) frameData);
				break;
			case M600_Air::REV_MAC:
				missionActionChoice((MissionAction*) frameData);
				break; 
			default:
				break;
		}
	}


	//pub one FCtr Topic
	void pubFCtrTopic(FlyCtrData * frameData){
		uint8_t * temp = (uint8_t *)frameData;
		std::cout<<"byte3";
				for(int i = 0;i<18;i++){
					printf(" %x ",temp[i]);
				}
		std::cout<<std::endl;
		if(frameData ->emFlag == 1){
			flightKeyCtr((int)frameData->mode);
		}else if(frameData ->emFlag == 0){
			joyVelCtr.axes.push_back(frameData ->ref1/10);
			joyVelCtr.axes.push_back(frameData ->ref2/10);
			joyVelCtr.axes.push_back(frameData ->ref3/10);
			joyVelCtr.axes.push_back(frameData ->ref4/10);
			joyVelCtr.axes.push_back(10000.0);
			joyVelCtr.axes.push_back(-5000.0);
			fCtrPub.publish(joyVelCtr);
			joyVelCtr.axes.clear();
		}
	}


	void flightKeyCtr(int kind){
		dji_sdk::DroneTaskControl droneTaskControl;
		droneTaskControl.request.task = kind;
		drone_task_service.call(droneTaskControl);
		//sdk_ctrl_authority_service
	}

	
	void parseSystem(SystemStatus * frameData){
		std::cout<<"sys1111:"<<frameData->authority<<std::endl;
		if(frameData ->authority != 0){
			if(frameData->authority == 1){
				dji_sdk::SDKControlAuthority authority;
  				authority.request.control_enable=1;
				sdk_ctrl_authority_service.call(authority);
			}else if(frameData->authority ==2){
				dji_sdk::SDKControlAuthority authority;
  				authority.request.control_enable=0;
				sdk_ctrl_authority_service.call(authority);
			}
		}
	}
	
	//pub one GimCtr Topic
	void pubGimCtrTopic(GimCtrData * frameData){
		gimCtr.pry.x = frameData->pitchCtr;
		gimCtr.pry.z = frameData->yawCtr;
		gimCtr.mutiple.data = frameData->setMutiple;
		gimCtr.setFcus.data = frameData->setFocus;
		gimCtr.reset.data = frameData->reset;
		gimCtr.sos.data = frameData->camSOS;
		gimCtr.sor.data = frameData->camSOR;
		gimCtrPub.publish(gimCtr);
	}


	//////////////////////////////////////wayMission
	
	//upload
	void byteDataToWPMFrame(uint8_t * keyData){

		WaypointMission * revWPMission = new WaypointMission();
		revWPMission->latLntList = new int32_t*[keyData[1]];
		for(int i = 0;i<keyData[1];i++){
			revWPMission->latLntList[i] = new int32_t[2];
		}
		revWPMission->waypointAction = new uint8_t*[keyData[1+8*keyData[1]+1]];
		for(int i = 0;i< keyData[1+8*keyData[1]+1];i++){
			revWPMission->waypointAction[i] = new uint8_t[2];
		}
		memmove(&(revWPMission->missionIndex),&keyData[0],2);
		for(int i = 0 ;i<revWPMission->latLntCount;i++){
			memmove(&(revWPMission->latLntList[i][0]),&keyData[2+i*8],4);
			memmove(&(revWPMission->latLntList[i][1]),&keyData[2+i*8+4],4);
			//std::cout<<(double)(revWPMission->latLntList[i][0]/(pow(10,7)*1.0))<<"  "<<(double)(revWPMission->latLntList[i][1]/(pow(10,7)*1.0))<<std::endl;
		}
		memmove(&(revWPMission->actionCount),&keyData[1+8*revWPMission->latLntCount+1],1);
		for(int i = 0;i<revWPMission->actionCount;i++){
			memmove(&(revWPMission->waypointAction[i][0]),
			&keyData[1+8*revWPMission->latLntCount+1+1+i*2],1);
			memmove(&(revWPMission->waypointAction[i][1]),
			&keyData[1+8*revWPMission->latLntCount+1+1+i*2+1],1);
		}
		memmove(&(revWPMission->altitude),
		&keyData[1+8*revWPMission->latLntCount+1+2*revWPMission->actionCount+1],11);
		
		uploadWPM(revWPMission);			
	}	


	void  uploadWPM(WaypointMission* frameData){
		initWaypointTask(frameData);
		for(int i = 0;i<frameData->actionCount;i++){
			wayPointAction.action_repeat = 1;
			wayPointAction.command_list[i]=frameData->waypointAction[i][0];
			wayPointAction.command_parameter[i]=frameData->waypointAction[i][1];	
		}
		for(int i = 0;i < frameData->latLntCount ;i++){
			std::cout<<frameData->latLntList[i][0]<<"  "<<frameData->latLntList[i][1]<<std::endl;
			waypoint.latitude = (double)frameData->latLntList[i][0];
			waypoint.longitude = (double)frameData->latLntList[i][1];
			waypoint.latitude = waypoint.latitude /pow(10,7);
			waypoint.longitude = waypoint.longitude /pow(10,7);
			std::cout<<std::setprecision(7)<<std::fixed<<waypoint.latitude<<"  "<<std::setprecision(7)<<std::fixed<<waypoint.longitude<<std::endl;
			waypoint.altitude = frameData->altitude;
			waypoint.damping_distance    = 0;
			waypoint.target_yaw          = 0;
			waypoint.target_gimbal_pitch = 0;
			waypoint.turn_mode           = 0;
			waypoint.has_action          = 0;
			waypoint.waypoint_action = wayPointAction;
			waypointTask.mission_waypoint.push_back(waypoint);
		}
		if(initWaypointMission( ).result){
			ROS_INFO("Waypoint upload command sent successfully");
		}
		else{
			ROS_WARN("Failed sending waypoint upload command");
		}		
	}


	void initWaypointTask(WaypointMission * frameData){
		waypointTask.velocity_range     = 15;
		waypointTask.idle_velocity      = frameData->velocity;
		switch ((int)frameData->finishAction){
			case 1:
				waypointTask.action_on_finish  = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
				break;
			case 2:
				break;
			case 3:
				break;
			case 4:
				break;
			default:
				break;
		}
		waypointTask.mission_exec_times = frameData->repeatTimes;	
		switch ((int)frameData->headingMode){
			case 1:
				waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
				break;
			case 2:
				break;
			case 3:
				break;
			case 4:
				break;
			default:
				break;
		}
		waypointTask.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
		waypointTask.action_on_rc_lost  = dji_sdk::MissionWaypointTask::ACTION_AUTO;
		waypointTask.gimbal_pitch_mode  = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
	}

	ServiceAck initWaypointMission( ){
		dji_sdk::MissionWpUpload missionWpUpload;
		missionWpUpload.request.waypoint_task = waypointTask;
		waypoint_upload_service.call(missionWpUpload);
		if (!missionWpUpload.response.result){
			ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set,
			missionWpUpload.response.cmd_id);
			ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
		}
		return ServiceAck(
		missionWpUpload.response.result, missionWpUpload.response.cmd_set,
		missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data);
	}

	//////////////////////////////////////////////////////////////////////missionAction



	void missionActionChoice(MissionAction * frameData){
		switch ((int) frameData->missionType){
			case 1:
				switch((int)frameData->actionType){
					case 1:
						missionAction(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT,
						DJI::OSDK::MISSION_ACTION::START);
						break;
					case 2:
						missionAction(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT,
						DJI::OSDK::MISSION_ACTION::STOP);
						break;
					case 3:
						missionAction(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT,
						DJI::OSDK::MISSION_ACTION::PAUSE);
						break;
					case 4:
						missionAction(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT,
						DJI::OSDK::MISSION_ACTION::RESUME);
						break;
				}
				break;
			case 2:
				switch((int)frameData->actionType){
					case 1:
						missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
						DJI::OSDK::MISSION_ACTION::START);
						break;
					case 2:
						missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
						DJI::OSDK::MISSION_ACTION::STOP);
						break;
					case 3:
						missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
						DJI::OSDK::MISSION_ACTION::PAUSE);
						break;
					case 4:
						missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
						DJI::OSDK::MISSION_ACTION::RESUME);
						break;
				}
				break;
		}
		
	
	}


	ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type, DJI::OSDK::MISSION_ACTION action){
		dji_sdk::MissionWpAction missionWpAction;
		dji_sdk::MissionHpAction missionHpAction;
		switch (type){
			case DJI::OSDK::WAYPOINT:
				missionWpAction.request.action = action;
				waypoint_action_service.call(missionWpAction);
				if (!missionWpAction.response.result){
					ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set,
					missionWpAction.response.cmd_id);
					ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
				}
      				return {missionWpAction.response.result,
               				missionWpAction.response.cmd_set,
               				missionWpAction.response.cmd_id,
              	 			missionWpAction.response.ack_data};
			case DJI::OSDK::HOTPOINT:
				bool takeoff_succed = false;
 					// Takeoff
				if (takeoff().result){
    					ROS_INFO("Takeoff command sent successfully");
				}
				else{
					ROS_WARN("Failed sending takeoff command");
					return ServiceAck(0,0,0,0);
				}
				ros::Duration(15).sleep();
				missionHpAction.request.action = action;
				hotpoint_action_service.call(missionHpAction);
				if (!missionHpAction.response.result){
					ROS_WARN("ack.info: set = %i id = %i", missionHpAction.response.cmd_set,
					missionHpAction.response.cmd_id);
					ROS_WARN("ack.data: %i", missionHpAction.response.ack_data);
				}
				return ServiceAck(
					missionHpAction.response.result, missionHpAction.response.cmd_set,
					missionHpAction.response.cmd_id, missionHpAction.response.ack_data);
		}
	}
    
	
	///////////////////////////////////////////////////////////////////////hotpointMission

	void byteDataToHPMFrame(HotpointMission* frameData){
		hotpointTask.latitude      = frameData->latLnt[0]/7.0;
  		hotpointTask.longitude     = frameData->latLnt[1]/7.0;
		hotpointTask.altitude      = frameData->altitude;
		hotpointTask.radius        = frameData->radius;
  		hotpointTask.angular_speed = frameData->angleVel;
  		hotpointTask.is_clockwise  = 0;
 		hotpointTask.start_point   = frameData->startHotPoint;
  		hotpointTask.yaw_mode      = frameData->headingMode;
		uploadHPM();
	}
	

	ServiceAck initHotpointMission(){
		dji_sdk::MissionHpUpload missionHpUpload;
		missionHpUpload.request.hotpoint_task = hotpointTask;
		hotpoint_upload_service.call(missionHpUpload);
		return ServiceAck(
			missionHpUpload.response.result, missionHpUpload.response.cmd_set,
 			missionHpUpload.response.cmd_id, missionHpUpload.response.ack_data);
	}

	bool uploadHPM(){
		if(initHotpointMission( ).result){
			ROS_INFO("Hotsoint upload command sent successfully");
			return true;
		}
		else{
			ROS_WARN("Failed sending hotpoint upload command");
			return false;
		}	
	}
	

//////////////////////////////////////takeoff  service
	ServiceAck takeoff(){
		dji_sdk::DroneTaskControl droneTaskControl;
		droneTaskControl.request.task = 4;
		drone_task_service.call(droneTaskControl);
		if (!droneTaskControl.response.result){
			ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
			droneTaskControl.response.cmd_id);
			ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
		}
		return ServiceAck(
			droneTaskControl.response.result, droneTaskControl.response.cmd_set,
			droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
	}



	void toContinueShootThread(int shootVel){
		boost::thread t = boost::thread(boost::bind(&M600_Air::toContinueShoot,this,shootVel));
		//t.join();
	}


	void toMoveGimMHDThread(int mode,int degree){
		boost::thread t = boost::thread(boost::bind(&M600_Air::missionGimHD,this,mode,degree));			
		//t.join();
	}

	void toSingleShootThread(){
		boost::thread t = boost::thread(&M600_Air::toSingleShoot,this);
	}

	void totoRecordThread(){
		boost::thread t = boost::thread(&M600_Air::toRecord,this);
	}

	void missionGimHD(int mode,int degree){
		dji_sdk::GimCtr  mGimMCtr;
		gimHDMutex.lock();
		switch(mode){
			case 1:
				mGimMCtr.pry.y = 1;
				mGimMCtr.pry.x = degree;
				gimCtrPub.publish(mGimMCtr);
				break;
			case 2:
				mGimMCtr.pry.y = 1;
				mGimMCtr.pry.z = degree;
				gimCtrPub.publish(mGimMCtr);
				break;
			case 3:
				mGimMCtr.reset.data = 1;
				gimCtrPub.publish(mGimMCtr);
				break;
		}
		boost::this_thread::sleep(boost::posix_time::seconds(3));
		gimHDMutex.unlock();				
	}

	void toContinueShoot(int shootVel){
		dji_sdk::GimCtr  mGimMCtr;
		gimSrcMutex.lock(); 
		if(shootVel <3){
			std::cout<<"shootVel should not be larger than 3"<<std::endl;
			return ;
		}
		while(isContinueEndSrc){
			while(isContinueCanSrc){
				mGimMCtr.sos.data = 1;
				gimCtrPub.publish(mGimMCtr);
				boost::this_thread::sleep(boost::posix_time::seconds(3));
			}
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}
		isContinueEndSrc = true;
		gimSrcMutex.unlock();
	}


	void toSingleShoot(){
		dji_sdk::GimCtr  mGimMCtr;
		gimSrcMutex.lock(); 
			while(isSingleEndSrc){
				while(isSingleCanSrc){
					mGimMCtr.sos.data = 1;
					gimCtrPub.publish(mGimMCtr);
					boost::this_thread::sleep(boost::posix_time::seconds(3));
					isSingleCanSrc = false;
				}
				boost::this_thread::sleep(boost::posix_time::seconds(1));
			}
			isSingleEndSrc = true;
		gimSrcMutex.unlock();
	}


	void toRecord(){
		dji_sdk::GimCtr  mGimMCtr;
		gimSrcMutex.lock(); 
			while(isRecordEndSrc){
				while(isRecordCanSrc){
					mGimMCtr.sos.data = 1;
					gimCtrPub.publish(mGimMCtr);
					boost::this_thread::sleep(boost::posix_time::seconds(3));
					isRecordCanSrc =  false;
				}
				boost::this_thread::sleep(boost::posix_time::seconds(1));
			}
			isRecordEndSrc = true;
		gimSrcMutex.unlock();
	}



public:
	//class structer init
	M600_Air(ros::NodeHandle &nh){initMySerial();initSubPub(nh);revByteDataLoopThread();}
	~M600_Air(){mySerial->close();}


	//init p900 serial
	void initMySerial(){
		if(mySerial == NULL){
			mySerial = new serial::Serial();
		}
		mySerial->setPort("/dev/ttyUSB1");
		mySerial->setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    		mySerial->setTimeout(to);
    		mySerial->open();
		if (mySerial->isOpen()){
			ROS_INFO("Serial Port initialized");
		}
	};

};

int main(int argc ,char **argv){
	ros::init(argc,argv,"M600_Air");
	ros::NodeHandle nh;
	M600_Air* m600_air = new M600_Air(nh);	
	ros::spin();
	return 0;
}
	
