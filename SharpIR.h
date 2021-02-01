/*
 * SharpIR
 * Library for the acquisition of distance data from Sharp IR sensors
 */

//#pragma once
#ifndef ir_controller
#define ir_controller

#include <Arduino.h>

class SharpIR
	{
		public:

			using sensorCode = const uint8_t ;

			SharpIR( sensorCode _sensorType , uint8_t _sensorPin ) : sensorType( _sensorType ) , pin( _sensorPin ) {}
			//uint8_t
			float getDistance( bool avoidBurstRead = true ) ; //default true
			static sensorCode GP2Y0A41SK0F = 0 ;
			static sensorCode GP2Y0A21YK0F = 1 ; //we are using 5 of this sensor atm
			static sensorCode GP2Y0A02YK0F = 3 ; //using 1 of this

		protected:

			uint8_t sensorType, pin ;

		private:

			uint32_t lastTime = 0 ;
	};
#endif //end
