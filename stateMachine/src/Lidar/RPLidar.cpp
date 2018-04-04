/*
 * RoboPeak RPLIDAR Driver for Arduino
 * RoboPeak.com
 * 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */



#include "RPLidar.h"

RPLidar::RPLidar()
    : _bined_serialdev(NULL)
{
    _currentMeasurement.distance = 0;
    _currentMeasurement.angle = 0;
    _currentMeasurement.quality = 0;
    _currentMeasurement.startBit = 0;
}


RPLidar::~RPLidar()
{
    end();
}

// open the given serial interface and try to connect to the RPLIDAR
bool RPLidar::begin(SerialPort::Port port)
{
    if (isOpen()) {
      end(); 
    }

    //_bined_serialdev = serialobj;
    _bined_serialdev = new SerialPort(RPLIDAR_SERIAL_BAUDRATE,port);
   // _bined_serialdev->end();
    //_bined_serialdev->begin(RPLIDAR_SERIAL_BAUDRATE);
    _bined_serialdev->Reset();
    start = std::clock();
}

// close the currently opened serial interface
void RPLidar::end()
{
    if (isOpen()) {
     //  _bined_serialdev->end();
    	_bined_serialdev->~SerialPort();
       _bined_serialdev = NULL;
    }
}


// check whether the serial interface is opened
bool RPLidar::isOpen()
{
    return _bined_serialdev?true:false; 
}

// ask the RPLIDAR for its health info
u_result RPLidar::getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout)
{
    _u32 currentTs = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    _u32 remainingtime;
  
    _u8 *infobuf = (_u8 *)&healthinfo;
    _u8 recvPos = 0;

    rplidar_ans_header_t response_header;
    u_result  ans;


    if (!isOpen()) return RESULT_OPERATION_FAIL;

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0))) {
            return ans;
        }

        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
            return RESULT_INVALID_DATA;
        }

        if ((response_header.size) < sizeof(rplidar_response_device_health_t)) {
            return RESULT_INVALID_DATA;
        }
        
        while ((remainingtime=( std::clock() - start ) / (double) CLOCKS_PER_SEC - currentTs) <= timeout) {
        	char * currentbyte_in = new char;
        	_bined_serialdev->Read(currentbyte_in,1);
            int currentbyte = *currentbyte_in;//_bined_serialdev->read();
            if (currentbyte < 0) continue;
            
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_health_t)) {
                return RESULT_OK;
            }
        }
    }
    return RESULT_OPERATION_TIMEOUT;
}

// ask the RPLIDAR for its device info like the serial number
u_result RPLidar::getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout )
{
    _u8  recvPos = 0;
    _u32 currentTs = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    _u32 remainingtime;
    _u8 *infobuf = (_u8*)&info;
    rplidar_ans_header_t response_header;
    u_result  ans;

    if (!isOpen()) return RESULT_OPERATION_FAIL;

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO,NULL,0))) {
            return ans;
        }

        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        if (response_header.size < sizeof(rplidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        while ((remainingtime=( std::clock() - start ) / (double) CLOCKS_PER_SEC - currentTs) <= timeout) {
        	char * currentbyte_in = new char;
			_bined_serialdev->Read(currentbyte_in,1);
			int currentbyte = *currentbyte_in;//_bined_serialdev->read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_info_t)) {
                return RESULT_OK;
            }
        }
    }
    
    return RESULT_OPERATION_TIMEOUT;
}

// stop the measurement operation
u_result RPLidar::stop()
{
    if (!isOpen()) return RESULT_OPERATION_FAIL;
    u_result ans = _sendCommand(RPLIDAR_CMD_STOP,NULL,0);
    return ans;
}

// start the measurement operation
u_result RPLidar::startScan(bool force, _u32 timeout)
{
    u_result ans;

    if (!isOpen()) return RESULT_OPERATION_FAIL;
    
    stop(); //force the previous operation to stop

    {
       // ans = _sendCommand(force?RPLIDAR_CMD_FORCE_SCAN:RPLIDAR_CMD_SCAN, NULL, 0);
    	ans = _sendCommand(force?RPLIDAR_CMD_FORCE_SCAN:RPLIDAR_CMD_SCAN, NULL, 0);
        if (IS_FAIL(ans)) return ans;

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
            return RESULT_INVALID_DATA;
        }

        if (response_header.size < sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }
    }
    return RESULT_OK;
}

// wait for one sample point to arrive
u_result RPLidar::waitPoint(_u32 timeout)
{
   _u32 currentTs = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
   _u32 remainingtime;
   rplidar_response_measurement_node_t node;
   _u8 *nodebuf = (_u8*)&node;

   _u8 recvPos = 0;

   while ((remainingtime=( std::clock() - start ) / (double) CLOCKS_PER_SEC - currentTs) <= timeout) {
	   char * currentbyte_in = new char;
		_bined_serialdev->Read(currentbyte_in,1);
		int currentbyte = *currentbyte_in;//_bined_serialdev->read();
        if (currentbyte<0) continue;

        switch (recvPos) {
            case 0: // expect the sync bit and its reverse in this byte          {
                {
                    _u8 tmp = (currentbyte>>1);
                    if ( (tmp ^ currentbyte) & 0x1 ) {
                        // pass
                    } else {
                        continue;
                    }

                }
                break;
            case 1: // expect the highest bit to be 1
                {
                    if (currentbyte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        // pass
                    } else {
                        recvPos = 0;
                        continue;
                    }
                }
                break;
          }




          nodebuf[recvPos++] = currentbyte;

          if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
              // store the data ...
              _currentMeasurement.distance = node.distance_q2/4.0f;
              _currentMeasurement.angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
              _currentMeasurement.quality = (node.sync_quality>>RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
              _currentMeasurement.startBit = (node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
              return RESULT_OK;
          }
        
   }

   return RESULT_OPERATION_TIMEOUT;
}


void RPLidar::lidar_thread_internal ()
{
	while(1)
	{
		if (IS_OK(this->waitPoint(RPLIDAR_DEFAULT_TIMEOUT))) {};


		//angle_stuff[(int) _currentMeasurement.angle] =  _currentMeasurement;

		//they generally have about... 302, make it 304 to be safe.. 310 to be safe... but usually [300-302]. have a check to see if over 310.
		count++;
		if(_currentMeasurement.startBit)
		{
//			printf("scanline : %d \t %d \n",count,count_line);
			count = 0;
			count_line ++;
		}

		if(count < 300)
		{
			angle_stuff[count] = _currentMeasurement;
		}


		//printf("the distance around 0: %f\n",angle_stuff[0].distance);

		//printf("sca\n");
		/*std::thread c_th(&RPLidar::lidar_thread,this);
	//	std::thread c_th(&drv->lidar_thread,drv);
		c_th.detach();
	*/
	}
}


std::vector<RPLidarMeasurement> RPLidar::GetPointCloud(double angleLow, double angleHigh, bool throughZero)//maybe do something else to handle any jump. like at -pi = pi
{
	std::vector<RPLidarMeasurement> out;
	for(int i=0;i<300;i++)
	{
		if(throughZero ) // you wrap around zero
		{
			if(angle_stuff[i].angle < angleLow || angle_stuff[i].angle > angleHigh)
			{
				//good value
				out.push_back(angle_stuff[i]);
			}
		}
		else
		{
			if(angle_stuff[i].angle > angleLow && angle_stuff[i].angle < angleHigh)
			{
				//good value
				out.push_back(angle_stuff[i]);
			}
		}
	}
	return out;
}

void RPLidar::lidar_thread ()
{
	angle_stuff.resize(304);
	std::thread c_th(&RPLidar::lidar_thread_internal,this);
	c_th.detach();
}


u_result RPLidar::_sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{

    rplidar_cmd_packet_t pkt_header;
    rplidar_cmd_packet_t * header = &pkt_header;
    _u8 checksum = 0;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // send header first
    _bined_serialdev->Write( (char *)header, 2);

    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((_u8 *)payload)[pos];
        }

        // send size
        _u8 sizebyte = payloadsize;
        _bined_serialdev->Write((char *)&sizebyte, 1);

        // send payload
        _bined_serialdev->Write((char *)&payload, sizebyte);

        // send checksum
        _bined_serialdev->Write((char *)&checksum, 1);

    }

    return RESULT_OK;
}

u_result RPLidar::_waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout)
{
    _u8  recvPos = 0;
    _u32 currentTs = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    _u32 remainingtime;
    _u8 *headerbuf = (_u8*)header;
    while ((remainingtime=( std::clock() - start ) / (double) CLOCKS_PER_SEC - currentTs) <= timeout) {
        
    	char * currentbyte_in = new char;
		_bined_serialdev->Read(currentbyte_in,1);
		int currentbyte = *currentbyte_in;//_bined_serialdev->read();
        if (currentbyte<0) continue;
        switch (recvPos) {
        case 0:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) {
                continue;
            }
            break;
        case 1:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) {
                recvPos = 0;
                continue;
            }
            break;
        }
        headerbuf[recvPos++] = currentbyte;

        if (recvPos == sizeof(rplidar_ans_header_t)) {
           // printf("%d\n",remainingtime);
        	return RESULT_OK;
        }
  

    }

    return RESULT_OPERATION_TIMEOUT;
}

void clamp (int& a, int l, int h)
{
	if(a<l)
		a=l;
	if(a>h)
		a=h;
}

void clamp (double& a, double l, double h)
{
	if(a<l)
		a=l;
	if(a>h)
		a=h;
}

int wrap(int a, int bounds)
{
	if(a < 0)
		return bounds + a;
	if(a >= bounds)
		return a - bounds;
	return a;
}

RPLidarMeasurement RPLidar::getRoughAngleDistance(int angle)
{
	clamp(angle,0,360);
	int index = (int)((angle / 360.0) * 300.0); //maps the value from [0-360] to [0-300]
	return angle_stuff[index];
}

RPLidarMeasurement RPLidar::getCloseAngleDistance(double angle)
{
	clamp(angle,0,360);
	double angle_ = (double)angle;
	//now we need to check the surrounding one until we find one that matches
	int i = (int)((angle / 360.0) * 300.0);	//best first estimate
	while(1)		//yes. this is infinite, but...  who cares?
	{
		double low_err = fabs(angle_ - angle_stuff[wrap(i - 1,300)].angle); 	//error between lower angle and goal angle
		double high_err = fabs(angle_ - angle_stuff[wrap(i + 1,300)].angle); 	//error between higher angle and goal angle
		double cur_err = fabs(angle_ - angle_stuff[i].angle); 					//error between this angle and goal angle
		if(cur_err > low_err || cur_err > high_err)		//if any of the other are better
		{
			//see which error is less
			if(low_err < high_err)
			{
				//lower is better estimate
				i = wrap(i-1,300);
			}
			else
			{
				//higher is better estimate
				i = wrap(i+1,300);
			}
			//then it goes through again...
		}
		else
		{
			//we are in the best one...
			return angle_stuff[i];
		}
	}
//	return RPLidarMeasurement;		//is just a null, but idk how to make a null RPLidarMeasurement
}

RPLidarMeasurement RPLidar::getAvgAngleDistance(int angle, int range)
{
	clamp(angle,0,360);
	int angle_index = (int)((angle / 360.0) * 300.0); //maps the value from [0-360] to [0-300]
	RPLidarMeasurement out;
	out.angle = 0;
	out.distance = 0;
	for(int i = angle_index - range; i < angle_index + range;i++)
	{
		int j = wrap(i,300);
		out.distance += angle_stuff[j].distance;
		out.angle += angle_stuff[j].angle;
	}
	out.distance /= (2.0 * range + 1.0); // gets the average
	out.angle /= (2.0 * range + 1.0); // gets the average
	return out;
}

double RPLidar::getRangeDistance(double angleLow, double angleHigh, bool throughZero)
{
	std::vector<RPLidarMeasurement> vals = GetPointCloud(angleLow,angleHigh,throughZero);
	double out= 0;
	//printf("how large%d\n",vals.size());
	for(int i=0;i<vals.size();i++)
	{
		out += vals[i].distance;
	}
	out = out / ((double)vals.size());
	return out;
}

void RPLidar::printValues()
{
//	for(int i=0;i<300;i++)
//	{
//		printf("%d : %d \n",angle_stuff[i].angle,angle_stuff[i].distance);
//	}
	//if(angle_stuff[0])
		printf("%f : %f \n",angle_stuff[0].angle,angle_stuff[0].distance);
}

