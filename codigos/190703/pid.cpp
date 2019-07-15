#include <cmath>
#include <math.h>
#include <MS5611.h>
#include <Util.h>
#include <unistd.h>
#include <stdio.h>
#include "Nextion.h"
#include "RCOutput_Navio2.h"
#include "PWM.h"
#include <memory>
#include <string>
#include <sys/time.h>
#include "MPU9250.h"
#include "LSM9DS1.h"
#include <pthread.h>
#include <ADC_Navio2.h>

#define SERVO_MIN 1000 /*mS*/
#define SERVO_MAX 2000 /*mS*/
#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1
#define PWM_OUTPUT_3 2
#define PWM_OUTPUT_4 3



RCOutput_Navio2 servo1;
MPU9250 sensormpu;
ADC_Navio2 ADC;

float comando_1 = 1000;
float comando_2 = 1000;
float comando_3 = 1000;
float comando_4 = 1000;

float axmpu, aympu, azmpu;
float gxmpu, gympu, gzmpu;
float mxmpu, mympu, mzmpu;

float angulo_x =  0;
float angulo_y =  0;
float angulo_z =  0;
float gx = 0;
float gy = 0;
float gz = 0;
float angz = 0;
float somador = 0;

struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;

float ch0 = 0;
float ch1 = 0;
float ch2 = 0;
float ch3 = 0;
float ch4 = 0; //CANAIS DA PORTA ADC NA NAVIO2 
float ch5 = 0; //CANAIS DA PORTA ADC NA NAVIO2

float pot1=0;



int main()
{

    if (check_apm()) 
    {
        return 1;
    }

    ADC.initialize();

    sensormpu.initialize();

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

    servo1.set_duty_cycle(PWM_OUTPUT_1, 1000);
    servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
    servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);
    servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);

    ch5 = ADC.read(5);
    pot1 = ch5/1000;
    float ed = (93.066*pot1) - 28.598 - 80;
    float xd = 0, xd_ponto, c = 0.2, wd = 0 ;
    float intervalo = 20;
    float flag = 1;
    float teta_integrado = 0;

    float P=0,I=0,D=0,PID=0;
    float setpoint0 = 15; //graus
     float setpoint; //graus
    float kp   = 0.01;
    float ki   = 0.02;
    float kd   = 0.00;;
    float e = 0; 
    float teta = 0; 
    float xi = 0;
    float wi = 0;
    float u = 0;
    float pmx = 1408;
    float pmi = 1192;
    float T = 0.5;
    teta = (93.066*pot1) - 28.598 - 80;
    float tetaf = teta;
    float up,ui,ud;

    while(true)
    {
        
        gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        if (dt>100)
        {
            dt = 0;
        }

        segundos = segundos+dt;

        setpoint=setpoint0*(1-exp(-segundos/1));

        ch5 = ADC.read(5); //ADC3 DA PORTA ADC NA NAVIO2
        pot1 = ch5/1000;
        teta = (93.066*pot1) - 28.598 - 80;
        tetaf = tetaf + (-tetaf/T + teta/T)*dt;
        e = setpoint - teta;
        xi = xi + e*dt;
        wi = xi;

        xd_ponto = (-1/c)*xd + e;
        xd       = xd + xd_ponto*dt;
        wd       = (-1/(c*c))*xd + (1/c)*e;

        if(segundos>0.3)
        {
        	u = ki*wi + kd*wd + kp*e;
		}
		else
		{
			u = ki*wi + kp*e;
		}
		up= kp*e;
		ui=ki*wi;
		ud=kd*wd;

        comando_1 = (pmx+pmi)/2  + (pmx-pmi)*u/2;
        comando_3 = (pmx+pmi)/2  - (pmx-pmi)*u/2;


        //CONTROLADOR PID

        //xd_ponto = (-1/c)*xd +ed;
        //xd       = xd + xd_ponto*dt;
        //ed       = (93.066*pot1) - 28.598 - 80;
        //wd       = (-1/(c*c))*xd + (1/c)*ed;
        //D = wd*kd;

        //erro = setpoint - ed;

 		//P = erro*kp;

 		//I = ki*(I + erro*dt);

 		//PID = P + I + D;
        
        //controlando motores

        //LIMITADORES:
        if(u>1)
        	{u=1;}


        if(u<-1)
        	{u=-1;}


        servo1.set_duty_cycle(PWM_OUTPUT_1, comando_1);
        servo1.set_duty_cycle(PWM_OUTPUT_3, comando_3);

        servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
        servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);

        printf("TETA = %.2f TETAF %.2f erro = %.2f PWM1 = %.2f PWM2 = %.2f up= %.2f ui= %.2f ud= %.2f\n",teta,tetaf,e,comando_1,comando_3,up,ui,ud);
        usleep(20000);

    }
    return 0;
}