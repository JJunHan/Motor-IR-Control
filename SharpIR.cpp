#include "SharpIR.h"
#include <RunningMedian.h>
constexpr int SIZE = 50;
RunningMedian samples = RunningMedian(SIZE);

float SharpIR::getDistance( bool avoidBurstRead )
	{
		float distance ;
		int raw_data = 0;
    samples.clear();
		if( !avoidBurstRead ) while( millis() <= lastTime + 30 ) {} //wait for sensor's sampling time (26hz) 

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
        
				case A0: //Left side front sensor
          distance = 8314.53/(raw_data+46.96) -12;
          return distance;
					break;

				case A1: //middle front sensor
          distance = 7992.86/(raw_data+29.78) -12;
          return distance;
          
					break;

				case A2: //Left side left front sensor
        if(raw_data > 585) { //3,2,1 cm
          distance = -0.0938*raw_data+59.344; 
        }
        else{
          distance = 8216.3/(raw_data+51.71) -11;
        }
          return distance;
					break;

				case A3: //Left side left back sensor
          //if(raw_data > 585) { //3,2,1 cm
          //  distance = -0.0938*raw_data+59.344;
          //}
          //else{
            distance = 7665.63/(raw_data+26.11) -11; //665.63/(raw_data+26.11) -11
          //}
          
          return distance;
					break;
				
				case A4: //Right side front sensor
          distance = 8329.85/(raw_data+48.73) -12;
          return distance;
					break;

				}


			case GP2Y0A02YK0F : //used as long range A5
        samples.clear();
        for (int i = 0; i < SIZE; i++) {
          samples.add(analogRead(pin));
        }
        raw_data = samples.getMedian();
        distance = 17387.05/(raw_data+69.1) -19;
        return distance;
        break;
        
		}
	}
