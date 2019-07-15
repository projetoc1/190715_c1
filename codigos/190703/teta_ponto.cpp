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
    FILE *texto;
    texto = fopen("dados.txt","w");

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
    float xd = 0, xd_ponto, c = 0.1, wd = 0 ;
    float intervalo = 20;
    float flag = 1;
    float teta_integrado = 0;

    while(flag==1)
    {
        
        ch5 = ADC.read(5); //ADC3 DA PORTA ADC NA NAVIO2
        pot1 = ch5/1000;

        

        gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        if (dt>100)
        {
            dt = 0;
        }

        segundos = segundos+dt;

        if (segundos>=intervalo)
        {
            flag = 0;
        }
 		
        xd_ponto = (-1/c)*xd +ed;
        xd       = xd + xd_ponto*dt;
        ed       = (93.066*pot1) - 28.598 - 80;
        wd       = (-1/(c*c))*xd + (1/c)*ed;
        if(segundos>=0.1)
        {
        	teta_integrado = wd*dt + teta_integrado;
        }
        


        servo1.set_duty_cycle(PWM_OUTPUT_1, 1000);
        servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);

        servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
        servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);

        usleep(20000);
        fprintf(texto,"%.6f,%.2f,%.2f,%.2f\n",segundos,ed,wd,teta_integrado);
        printf("TEMPO = %.6f TETA = %.2f TETA_PONTO = %.2f TETA_INT = %.2f\n",segundos,ed,wd,teta_integrado);

    }
    fclose(texto);
    return 0;
}