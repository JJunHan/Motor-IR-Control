#include "SharpIR.h"
#include <RunningMedian.h>
constexpr int SIZE = 50;
RunningMedian samples = RunningMedian(SIZE);

float SharpIR::getDistance( bool avoidBurstRead )
	{
		float distance ;
		int raw_data;
		if( !avoidBurstRead ) while( millis() <= lastTime + 20 ) {} //wait for sensor's sampling time

		lastTime = millis();

		switch( sensorType )
		{
			case GP2Y0A41SK0F : //not used

				distance = 2076/(analogRead(pin)-11);

				if(distance > 30) return 31;
				else if(distance < 4) return 3;
				else return distance;

				break;

			case GP2Y0A21YK0F :
				samples.clear();
				for (int i = 0; i < SIZE; i++) {
					samples.add(analogRead(pin));
				}
				raw_data = samples.getMedian();
				//return raw_data;
        
				switch (pin) //A0-A4 total 5 sensors used
				{
        
				case A0: //Right side front sensor
        
					if (raw_data >= 490) {
						distance = -0.0272 * raw_data + 18.445;
					}
					else if (raw_data >= 338) {
						distance = -0.037 * raw_data + 22.344;
					}
					else if (raw_data >= 250) {
						distance = -0.0775 * raw_data + 35.264;
					}
					else if (raw_data >= 198) {
						distance = -0.1189 * raw_data + 45.098;
					}
					else if (raw_data >= 154) {
						distance = -0.1718 * raw_data + 55.08;
					}
					else if(raw_data >= 121) {
						distance = -0.2547 * raw_data + 67.883;
					}
					else if (raw_data >= 90) {
						distance = -0.3297 * raw_data + 78.631;
					}
					else {
						distance = -0.5685 * raw_data + 99.003;
					}
					return distance;
          
					break;

				case A1: //Left side front sensor
					if (raw_data >= 489) {
						distance = -0.0271 * raw_data + 18.356;
					}
					else if (raw_data >= 360) {
						distance = -0.0324 * raw_data + 20.331;
					}
					else if (raw_data >= 275) {
						distance = -0.0596 * raw_data + 30.007;
					}
					else if (raw_data >= 192) {
						distance = -0.1189 * raw_data + 45.458;
					}
					else if (raw_data >= 155) {
						distance = -0.1723 * raw_data + 54.638;
					}
					else if (raw_data >= 119) {
						distance = -0.2562 * raw_data + 67.277;
					}
					else if (raw_data >= 82) {
						distance = -0.2972 * raw_data + 73.114;
					}
					else {
						distance = -1.1427 * raw_data + 134.8;
					}
					return distance;

					break;

				case A2: //Left side left front sensor
					if (raw_data >= 505) {
						distance = -0.0297 * raw_data + 20.188;
					}
					else if (raw_data >= 372) {
						distance = -0.0315 * raw_data + 20.61;
					}
					else if (raw_data >= 290) {
						distance = -0.0511 * raw_data + 27.701;
					}
					else if (raw_data >= 234) {
						distance = -0.0959 * raw_data + 40.37;
					}
					else if (raw_data >= 187) {
						distance = -0.1287 * raw_data + 47.905;
					}
					else if (raw_data >= 150) {
						distance = -0.1912 * raw_data + 59.61;
					}
					else if (raw_data >= 126) {
						distance = -0.3165 * raw_data + 78.093;
					}
					else if (raw_data >= 100) {
						distance = -0.5169 * raw_data + 103.18;
					}
					else {
						distance = -0.7138 * raw_data + 123.96;
					}
					return distance;

					break;

				case A3: //Left side left back sensor
					if (raw_data >= 491) {
						distance = -0.0259 * raw_data + 17.882;
					}
					else if (raw_data >= 364) {
						distance = -0.0339 * raw_data + 21.242;
					}
					else if (raw_data >= 286) {
						distance = -0.0552 * raw_data + 28.71;
					}
					else if (raw_data >= 230) {
						distance = -0.0914 * raw_data + 39.057;
					}
					else if (raw_data >= 186) {
						distance = -0.1324 * raw_data + 48.435;
					}
					else if (raw_data >= 156) {
						distance = -0.2017 * raw_data + 61.354;
					}
					else if (raw_data >= 130) {
						distance = -0.3141 * raw_data + 78.588;
					}
					else if (raw_data >= 97) {
						distance = -0.4199 * raw_data + 91.976;
					}
					else {
						distance = -0.8544 * raw_data + 130.16;
					}
					return distance;

					break;
				
				case A4: //Right side right front senor
					if (raw_data >= 485) {
						distance = -0.0262 * raw_data + 17.771;
					}
					else if (raw_data >= 360) {
						distance = -0.0348 * raw_data + 21.418;
					}
					else if (raw_data >= 288) {
						distance = -0.0576 * raw_data + 29.452;
					}
					else if (raw_data >= 220) {
						distance = -0.0908 * raw_data + 38.677;
					}
					else if (raw_data >= 179) {
						distance = -0.1563 * raw_data + 52.813;
					}
					else if (raw_data >= 142) {
						distance = -0.199 * raw_data + 60.19;
					}
					else if (raw_data >= 117) {
						distance = -0.2797 * raw_data + 71.196;
					}
					else if (raw_data >= 89) {
						distance = -0.5343 * raw_data + 98.967;
					}
					else {
						distance = -0.5791 * raw_data + 102.2;
					}
					return distance;

					break;

				}


			case GP2Y0A02YK0F : //used as long range
        samples.clear();
        for (int i = 0; i < SIZE; i++) {
          samples.add(analogRead(pin));
        }
        raw_data = samples.getMedian();
        
        if (raw_data >= 465) {
          distance = -0.0921 * raw_data + 52.639;
        }
        else if (raw_data >= 407) {
          distance = -0.0909 * raw_data + 51.727;
        }
        else if (raw_data >= 342) {
          distance = -0.1046 * raw_data + 56.958;
        }
        else if (raw_data >= 300) {
          distance = -0.1584 * raw_data + 74.46;
        }
        else if (raw_data >= 246) {
          distance =  -0.1357 * raw_data +66.592;
        }
        else if (raw_data >= 210) {
          distance =  -0.214 * raw_data + 85.001;
        }
        else if (raw_data >= 182) {
          distance =  -0.3522 * raw_data + 113.16;
        }
        else if (raw_data >= 150) {
          distance = -0.6567 * raw_data + 166.77;
        }
        else {
          distance = -2.1677 * raw_data + 390.74;
        }
        return distance;
		}
	}
