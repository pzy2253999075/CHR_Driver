#include<dji_sdk/GasCtr.h>
//#include <dji_sdk/dji_sdk_node.h>
#include<serial/serial.h>
#include<ros/ros.h>

class SensorGas{
private:
	
	/*
	序号：	1		2		3		4		5		6		7		8		9		10		11		12
	名称：	CO		SO2		NO2		O3		VOC		空      CO2	   PM1.0    PM2.5   PM10   温度	  湿度
	小数：	2		3		3		3		3		0		0		0		0		0		1		0
	单位： ppm		ppm		ppm		ppm		ppm				ppm		ug/m3	ug/m3	ug/m3	摄氏度	  %RH
	*/
	//传感器数量 
	uint8_t GasNumDataSend[8]={0x01,0x03,0x00,0xF1,0x00,0x01,0xD5,0xF9};  
	
	//内部 6 组 7NE 传感器信息数据
	uint8_t StatusDataSend[8]={0x01,0x03,0x03,0x00,0x00,0x1E,0xC5,0x86}; 
	
	//实时数据 03 80
	uint8_t RTDataSend[8]={0x01,0x03,0x03,0x80,0x00,0x0C,0x44,0x63}; 
	
	//GPS数据 20 00
	uint8_t GPSDataSend[8]={0x01,0x03,0x20,0x00,0x00,0x01,0x8F,0xCA}; 
	
	serial::Serial* mySerial = NULL;
	dji_sdk::GasCtr mGasCtr;
	ros::Publisher GasCtrPub;
	ros::Rate focusTime = 10;
	ros::Rate zoomTime =1;
	pthread_t sendGassThread;
	ros::Rate send_data_rate = 5;
	int break_int[10]={0};
	
	void read_RTData(void)
	{
		long break_100ms = 0;
		
		
		uint8_t * dataBuffer = new uint8_t[255];
		
		//清除串口数据
		int len = mySerial->available();
		if(len>0)
		{
			mySerial->read(&dataBuffer[0],len);
		}
		
		//发送请求数据
		mySerial->write(RTDataSend,8);
		
		//读取串口接收数据
		while(1)
		{
			len = mySerial->available();
		
			if(len>=29)
			{
				
				//读取数据
				mySerial->read(&dataBuffer[0],len);
				
				//帧头判断
				if(dataBuffer[0]!=0x01 || dataBuffer[1]!=0x03 || dataBuffer[2]!=0x18)
				{
					break;
				}
			
				//CRC校验
				uint8_t CRCL,CRCH;
				if(dataBuffer[27]!=CRCL || dataBuffer[28]!=CRCH)
				{
					//break;
				}
				
				//提取数据
				mGasCtr.CO	= (dataBuffer[3]*256 + dataBuffer[4])*0.01;
				mGasCtr.SO2	= (dataBuffer[5]*256 + dataBuffer[6])*0.001;
				mGasCtr.NO2	= (dataBuffer[7]*256 + dataBuffer[8])*0.001;
				mGasCtr.O3	= (dataBuffer[9]*256 + dataBuffer[10])*0.001;
				mGasCtr.VOC	= (dataBuffer[11]*256 + dataBuffer[12])*0.001;
				//mGasCtr.CO= (dataBuffer[13]*256 + dataBuffer[14]);
				mGasCtr.CO2	= (dataBuffer[15]*256 + dataBuffer[16]);
				mGasCtr.PM1	= (dataBuffer[17]*256 + dataBuffer[18]);
				mGasCtr.PM2	= (dataBuffer[19]*256 + dataBuffer[20]);
				mGasCtr.PM10= (dataBuffer[21]*256 + dataBuffer[22]);
				mGasCtr.Temperature= (dataBuffer[23]*256 + dataBuffer[24])*0.1;
				mGasCtr.Humidity= (dataBuffer[25]*256 + dataBuffer[26])*0.1;
				GasCtrPub.publish(mGasCtr);
											
				//测试用，后期待删除
				std::cout<<break_int[0]++<<std::endl;
				break;
			}
			
			break_100ms++;
			if(break_100ms>1000000)	//大约0.5s
			{
				std::cout<<"read_RTData error!!!"<<std::endl;
				break;
			}
			
		}

	}
	
	static void* sendGassDataLoopStatic(void* object){
		reinterpret_cast<SensorGas*>(object)->sendGassDataLoop();
		return 0;
	}

  ////等各个线程退出后，进程才结束，否则进程强制结束了，线程可能还没反应过来；
    //pthread_exit(NULL);
	void sendGassDataLoopThread(){
		int result = pthread_create(&sendGassThread,NULL,&SensorGas::sendGassDataLoopStatic,this);
		if(result != 0){
			std::cout<<"pthread_create error:error_code ="<<result<<std::endl;
		}
	}	

	// 5 HZ
	void sendGassDataLoop(){
		while(ros::ok()){
			
			//测试用，后期待删除
			//std::cout<<"test"<<std::endl;

			read_RTData();
			
			send_data_rate.sleep();
		}	
	}
	

	// init Sub and Pub
	void initSubPub(ros::NodeHandle &nh){
		GasCtrPub = nh.advertise<dji_sdk::GasCtr>("/m600_GasCtr",10);
	}
	
	// init serial
	void initMySerial(){
		if(mySerial == NULL){
			mySerial = new serial::Serial();
		}
		mySerial->setPort("/dev/ttyUSB0");
		mySerial->setBaudrate(9600);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    		mySerial->setTimeout(to);
    		mySerial->open();
		if (mySerial->isOpen()){
			ROS_INFO("Serial Port initialized");
		}
	}
	
public:
	
	SensorGas(ros::NodeHandle &nh){initMySerial();initSubPub(nh);sendGassDataLoopThread();}
	~SensorGas(){mySerial->close();}
	
};

int main(int argc ,char ** argv){
	ros::init(argc,argv,"M600_SensorGas");
	ros::NodeHandle nh;
	SensorGas* mSensorGas =  new SensorGas(nh);
	ros::spin();
	return 0;
}
