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

class PID{
public:
  double error;
  double sample;
  double lastSample;
  double kP, kI, kD;      
  double P, I, D;
  double pid;
  double setPoint;
  long lastProcess;

  PID(double _kP, double _kI, double _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
  
  void addNewSample(double _sample){
    sample = _sample;
  }
  
  void setSetPoint(double _setPoint){
    setPoint = _setPoint;
  }
  
  double process(float _deltaTime){
    
    error = setPoint - sample;
    //float deltaTime = (micros() - lastProcess) / 1000000.0;
    float deltaTime = _deltaTime;
    //lastProcess = micros();
    
    P = error * kP;
   
    //I = I + (error * kI) * deltaTime;
    I = I + error*deltaTime;
    
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    
    pid = P + I*kI + D;
    
    return pid;
  }
};

RCOutput_Navio2 servo1,servo2,servo3,servo4;
MPU9250 sensormpu;
ADC_Navio2 ADC;

//PID pendulo(-0.06, 0.2, 0); //SETUP BOM NÃO PERFEITO

//PID pendulo(-0.06, 0.15, 0); //SETUP BOM E MELHOR
PID pendulo(0, 0.15, 0);
PID pendulo2(0.1 , 0 , 0 );


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

float aux1=0,aux2=0;


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
    //sleep(5);
    float referencia  = 1;
    float referencia2 = 1.25; 
    pendulo.setSetPoint(referencia);
    pendulo2.setSetPoint(referencia2);
    float comando_pid  = 0;
    float comando_pid2 = 0;

    while(true)
    {
        


        sensormpu.update();
        sensormpu.read_accelerometer(&axmpu, &aympu, &azmpu);
        sensormpu.read_gyroscope(&gxmpu, &gympu, &gzmpu);
        sensormpu.read_magnetometer(&mxmpu, &mympu, &mzmpu);

        gx = axmpu/9.81;
        gy = aympu/9.81;
        gz = azmpu/9.81;

        if (gx>1)
        {
            gx = 1;
        }

        if (gy>1)
        {
            gy = 1;
        }

        if (gz>1)
        {
            gz = 1;
        }

        angulo_x  = (180/3.1415)*asin(gx);
        angulo_y  = (180/3.1415)*acos(gy);
        angulo_z  = (180/3.1415)*asin(gz);
        if (contador>=2)
        {
            contador = 1;
            angz = somador/2;
            somador = 0;
        }
        else
        {
            somador = somador + angulo_z;
        }
        contador = contador + 1;
        //if (gx<0)
        //{
        //  angulo_x= angulo_x*(-1);
        //}

        if (gy<0)
        {
            angulo_y= angulo_y*(-1);
        }
        //if (gz<0)
        //{
       //   angulo_z= angulo_z*(-1);
        //}
        //aux1 = (10*angulo_z) + 200;
        //aux1 = (10*angulo_z) + 200;
        //aux2 = (10*angulo_y) + 200;

        //comando_1 = (aux1/2) +1200;
        //comando_3 = (-aux1/2) +1400;
        //comando_2 = (aux2/2) +1200;
        //comando_4 = (-aux2/2) +1400;

        //servo1.set_duty_cycle(PWM_OUTPUT_1, 1200);
        servo2.set_duty_cycle(PWM_OUTPUT_2, 1000);
        //servo3.set_duty_cycle(PWM_OUTPUT_3, 1200);
        servo4.set_duty_cycle(PWM_OUTPUT_4, 1000);
        
        float ch0 = ADC.read(0); //TENSÃO DA PLACA
        float ch1 = ADC.read(1); //TENSÃO DO BARRAMENTO DOS SERVOS
        float ch2 = ADC.read(2); //ADC0 DA PORTA DO POWER MODULE
        float ch3 = ADC.read(3); //ADC1 DA PORTA DO POWER MODULE
        float ch4 = ADC.read(4); //ADC2 DA PORTA ADC NA NAVIO2 
        float ch5 = ADC.read(5); //ADC3 DA PORTA ADC NA NAVIO2
        float pot1 = ch5/1000;
        float pot2 = ch4/1000;
        pendulo.addNewSample(pot1);
        pendulo2.addNewSample(pot2);

        gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        if (dt>100)
        {
            dt = 0;
        }

        segundos = segundos+dt;
        //pendulo.setSetPoint(1.15);
        if (segundos>10)
        {
            segundos = 0;
            if (referencia==1)
            {
                referencia = 1.3;
                pendulo.setSetPoint(referencia);

            }
            else
            {
                referencia = 1;
                pendulo.setSetPoint(referencia);
            }
        }

        //comando_pid varia de 0 a 100% sendo 50% o meio do pendulo
        comando_pid = pendulo.process(dt);
        comando_pid2 = pendulo2.process(dt);
        //if (comando_pid>100)
        //{
        //    comando_pid = 100;
        //}
        //if (comando_pid<0)
        //{
        //   comando_pid = 0;
        //}

        float aux_pid = comando_pid;
        float aux_pid2 = comando_pid2;

        comando_1 = 1300*(1+aux_pid);
        comando_3 = 1300*(1-aux_pid);
        comando_2 = 1300*(1+aux_pid2);
        comando_4 = 1300*(1-aux_pid2);

        if(comando_1>1400)
        	{comando_1=1400;}
        if(comando_3>1400)
        	{comando_3=1400;}
        if(comando_2>1400)
        	{comando_2=1400;}
        if(comando_4>1400)
        	{comando_4=1400;}

        if(comando_1<1000)
        	{comando_1=1000;}
        if(comando_3<1000)
        	{comando_3=1000;}
        if(comando_2<1000)
        	{comando_2=1000;}
        if(comando_4<1000)
        	{comando_4=1000;}

        //comando_1 = (aux_pid) +1300;
        //comando_3 = (-aux_pid) +1300;
        servo1.set_duty_cycle(PWM_OUTPUT_1, 1200);
        servo1.set_duty_cycle(PWM_OUTPUT_3, 1200);

        servo1.set_duty_cycle(PWM_OUTPUT_2, 1200);
        servo1.set_duty_cycle(PWM_OUTPUT_4, 1200);

        usleep(25000);
        printf("adcZ = %.2f PWM1 = %.2f PWM2 = %.2f pid = %.2f segundos = %.2f adcY = %.2f PWM3 = %.2f PWM4 = %.2f pid2 = %.2f\n",pot1,comando_1,comando_3, comando_pid,segundos,pot2,comando_2,comando_4,comando_pid2);
        //printf("gX = %.2f gY = %.2f gZ = %.2f aX = %.2f aY = %.2f aZ = %.2f\n",gx,gy,gz,angulo_x,angulo_y,angulo_z);
        //printf(" currenttime = %u segundos = %f dt = %f\n",currenttime, segundos, dt);
        //printf("comando 1 = %5.0f comando 3 = %5.0f angulo_z = %+3.0f\n",comando_1,comando_3,angulo_z);
    }

    return 0;
}