#include "RCOutput_Navio2.h"
#include "PWM.h"
#include "filtro.h"
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1
#define PWM_OUTPUT_3 2
#define PWM_OUTPUT_4 3

//PWM 
RCOutput_Navio2 servo1;

//AHRS
AHRS euler;

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;

int main()
{
    euler.sensorinit();
	//CONFIGURA 4 PORTAS DE PWM - freq 50Hz
    servo1.initialize(PWM_OUTPUT_1);
    servo1.set_frequency(PWM_OUTPUT_1, 50);
    servo1.enable(PWM_OUTPUT_1);

    servo1.initialize(PWM_OUTPUT_2);
    servo1.set_frequency(PWM_OUTPUT_2, 50);
    servo1.enable(PWM_OUTPUT_2);

    servo1.initialize(PWM_OUTPUT_3);
    servo1.set_frequency(PWM_OUTPUT_3, 50);
    servo1.enable(PWM_OUTPUT_3);

    servo1.initialize(PWM_OUTPUT_4);
    servo1.set_frequency(PWM_OUTPUT_4, 50);
    servo1.enable(PWM_OUTPUT_4);

    //angulos de euler
    float roll, pitch, yaw;
    euler.setGyroOffset();
    while(1)
    {
    servo1.set_duty_cycle(PWM_OUTPUT_1, 1300);
    servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
    servo1.set_duty_cycle(PWM_OUTPUT_3, 1300);
    servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);

    gettimeofday(&tv,NULL);
    previoustime = currenttime;
    currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
    dt = (currenttime - previoustime) / 1000000.0;
    //if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
    if (dt>100)
    {
        dt = 0;
    }

    euler.updateIMU(dt);

    euler.getEuler(&roll, &pitch, &yaw);
    printf("roll = %.2f pitch = %.2f yaw = %.2f\n",roll,pitch,yaw * -1);
    //float w = euler.getW();
    //float x = euler.getX();
    //float y = euler.getY();
    //float z = euler.getZ();
    //printf("w = %.2f x = %.2f y = %.2f z = %.2f\n",w,x,y,z);
    //usleep(50000);


    }

	return 0;
}