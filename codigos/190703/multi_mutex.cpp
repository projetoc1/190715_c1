
/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi

To run this example navigate to the directory containing it and run following commands:
make
./Barometer
*/
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
#include "MPU9250.h"
#include "LSM9DS1.h"
#include <pthread.h>

#define SERVO_MIN 1000 /*mS*/
#define SERVO_MAX 2000 /*mS*/
#define PWM_OUTPUT 0

MPU9250 sensormpu;
RCOutput_Navio2 servo1;

NexVariable setp_lat = NexVariable(0,37,"setp_lat");
NexVariable setp_lon = NexVariable(0,38,"setp_lon");

NexVariable lat_p = NexVariable(0,52,"lat_p");
NexVariable lat_i = NexVariable(0,53,"lat_i");
NexVariable lat_d = NexVariable(0,54,"lat_d");

NexVariable lon_p = NexVariable(0,55,"lon_p");
NexVariable lon_i = NexVariable(0,56,"lon_i");
NexVariable lon_d = NexVariable(0,57,"lon_d");

NexVariable error_lat = NexVariable(0,58,"error_lat");
NexVariable error_lon = NexVariable(0,59,"error_lon");

NexVariable flag = NexVariable(0,60,"flag");
NexVariable flag_piloto = NexVariable(0,63,"flag_piloto");

NexVariable lat_navio2 = NexVariable(0,67,"lat_navio2");
NexVariable lon_navio2 = NexVariable(0,68,"lon_navio2");

NexVariable adc1 = NexVariable(0,69,"adc1");
NexVariable adc2 = NexVariable(0,70,"adc2");

uint32_t setpoint_lat=0, setpoint_lon=0;
uint32_t latp=0,lati=0,latd=0;
uint32_t lonp=0,loni=0,lond=0;
float errorlat=3,errorlon=4;
uint32_t flag0=0,flagpiloto=0;
float latnavio2=1,lonnavio2=2;
float leituraadc1=0,leituraadc2=0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

float angulo=0;
float comando=1500;

pthread_mutex_t trava;
int contador_mutex=0;

void* f_tela(void* data)
{
    int cont = 1;
    while(1)
    {
        if(cont>15)
        {
            cont=1;
        }
        else
        {
            cont++;
        }
        //recebimento de variaveis
        //da navio2
        if (cont==1)
        {
            lat_navio2.setValue(angulo);
        }
        //lat_navio2.setValue(latnavio2);
        if (cont==2)
        {
            lon_navio2.setValue(lonnavio2);
        }
        //lon_navio2.setValue(lonnavio2);
        if (cont==3)
        {
            adc1.setValue(leituraadc1); 
        }
        //adc1.setValue(leituraadc1);
        if (cont==4)
         {
             adc2.setValue(leituraadc2);
         } 
        //adc2.setValue(leituraadc2);
         if (cont==5)
         {
             error_lat.setValue(errorlat);
         }
        //error_lat.setValue(errorlat);
         if (cont==6)
         {
             error_lon.setValue(errorlon);
         }
        //error_lon.setValue(errorlon);

        //LEITURA DA TELA
        if (cont==7)
        {
            setp_lat.getValue(&setpoint_lat);
        }
        //setp_lat.getValue(&setpoint_lat);
        if (cont==8)
        {
            setp_lon.getValue(&setpoint_lon);
        }
        //setp_lon.getValue(&setpoint_lon);

        if (cont==9)
        {
           lat_p.getValue(&latp);
        }
        //lat_p.getValue(&latp);
        if (cont==10)
        {
            lat_i.getValue(&lati);
        }
        //lat_i.getValue(&lati);
        if (cont==11)
        {
            lat_d.getValue(&latd);
        }
        //lat_d.getValue(&latd);
        if (cont==12)
        {
            lon_p.getValue(&lonp);
        }
        //lon_p.getValue(&lonp);
        if (cont==13)
        {
            lon_i.getValue(&loni);
        }
        //lon_i.getValue(&loni);
        if (cont==14)
        {
            lon_d.getValue(&lond);
        }
        //lon_d.getValue(&lond);
        if (cont==15)
        {
            flag_piloto.getValue(&flagpiloto);
        }
        //flag_piloto.getValue(&flagpiloto);
        printf("latp = %u lati = %u latd = %u\n",latp,lati,latd);
        //usleep(50000);
    }
}

void* mpu(void* data)
{
    while(1)
    {
        sensormpu.update();

        sensormpu.read_accelerometer(&ax, &ay, &az);
        sensormpu.read_gyroscope(&gx, &gy, &gz);
        sensormpu.read_magnetometer(&mx, &my, &mz);

        //printf("Acc: %+7.3f %+7.3f %+7.3f  ", ax, ay, az);
        //printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
        //printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);

        angulo = (180/3.1415)*acos(ax/9.81);
        comando = (2000/180)*angulo;
        servo1.set_duty_cycle(PWM_OUTPUT, comando);

        pthread_mutex_lock(&trava);
        contador_mutex=contador_mutex+100;
        pthread_mutex_unlock(&trava);
        //sleep(1);
    }
}

int main()
{
    pthread_t tela,leitura,servo;
    nexInit();

    //checa se a navio2 está funcional
    if (check_apm()) 
    {
        return 1;
    }
    //configura pwm na porta PWM_OUTPUT
    servo1.initialize(PWM_OUTPUT);
    servo1.set_frequency(PWM_OUTPUT, 50);
    servo1.enable(PWM_OUTPUT);
    //Inicializa mpu9250
    sensormpu.initialize();
    
    pthread_create(&leitura,NULL,mpu,NULL);
    pthread_create(&tela,NULL,f_tela,NULL);

    while (true)
    {
        pthread_mutex_lock(&trava);
        contador_mutex=contador_mutex+1;
        if(contador_mutex>=2000)
        {
            contador_mutex=0;
        }
        pthread_mutex_unlock(&trava);

        //printf("latp = %u lati = %u latd = %u\n",latp,lati,latd);
        //usleep(50000);
        //printf("contador_mutex = %d\n",contador_mutex);
    //    if (contador>=SERVO_MAX)
    //    {
    //        contador=SERVO_MIN;
    //        servo1.set_duty_cycle(PWM_OUTPUT, contador);
    //        //sleep(1);
    //    }

    //    latnavio2=contador;
    //    lonnavio2=contador;
    //    lat_navio2.setValue(latnavio2);
    //    lon_navio2.setValue(lonnavio2);
    //    servo1.set_duty_cycle(PWM_OUTPUT, comando);
    //    //printf("%f\n",angulo);
    //    contador=contador+100;        
    //    //sleep(0.1);
        //printf("loop principal\n");
        //sleep(2);
    }

    return 0;
}