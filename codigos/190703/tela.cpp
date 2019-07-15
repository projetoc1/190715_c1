//Bibliotecas
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

uint32_t setpoint_lat = 0;
uint32_t setpoint_lon = 0;
uint32_t latp  = 0;
uint32_t lati  = 0;
uint32_t latd  = 0;
uint32_t lonp  = 0;
uint32_t loni  = 0;
uint32_t lond  = 0;
float errorlat = 3;
float errorlon = 4;
uint32_t flag0 = 0;
uint32_t flagpiloto = 0;
float latnavio2 = 200;
float lonnavio2 = 2;
float leituraadc1 = 5;
float leituraadc2 = 6;


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
        //ENVIA PARA TELA NEXTION
        if (cont==1)
        {
            lat_navio2.setValue(latnavio2);
        }
        if (cont==2)
        {
            lon_navio2.setValue(lonnavio2);
        }
        if (cont==3)
        {
            adc1.setValue(leituraadc1); 
        }
        if (cont==4)
        {
           adc2.setValue(leituraadc2);
        } 
        if (cont==5)
        {
            error_lat.setValue(errorlat);
        }
        if (cont==6)
        {
            error_lon.setValue(errorlon);
        }
        //LEITURA DA TELA
        if (cont==7)
        {
           setp_lat.getValue(&setpoint_lat);
        }
        if (cont==8)
        {
            setp_lon.getValue(&setpoint_lon);
        }
        if (cont==9)
        {
            lat_p.getValue(&latp);
        }
        if (cont==10)
        {
            lat_i.getValue(&lati);
        }
        if (cont==11)
        {
            lat_d.getValue(&latd);
        }
        if (cont==12)
        {
            lon_p.getValue(&lonp);
        }
        if (cont==13)
        {
            lon_i.getValue(&loni);
        }
        if (cont==14)
        {
            lon_d.getValue(&lond);
        }
        if (cont==15)
        {
            flag_piloto.getValue(&flagpiloto);
        }
    }
}


//PROGRAMA PRINCIPAL

int main()
{   
    //TESTA FUNCIONAMENTO DA NAVIO2

    printf("LAT_P 1 = %.2f \n",latp);
    nexInit();
    printf("LAT_P 2 = %.2f \n",latp);

    while(true)
    {
        //lat_navio2.setValue(latnavio2);
        //lon_navio2.setValue(lonnavio2);
        //adc1.setValue(leituraadc1); 
        //adc2.setValue(leituraadc2);
        //error_lat.setValue(errorlat);
        //error_lon.setValue(errorlon);
        lat_p.getValue(&latp);
        lat_i.getValue(&lati);
        lat_d.getValue(&latd);
        lon_p.getValue(&lonp);
        lon_i.getValue(&loni);
        lon_d.getValue(&lond);
        flag_piloto.getValue(&flagpiloto);
        setp_lat.getValue(&setpoint_lat);

    	printf("LAT_P = %.2u LAT_I = %.2u LAT_D = %.2u LON_P = %.2u LON_I = %.2u LON_D = %.2u FLAG_PILOTO = %.2u SETPOINT = %.2u \n",latp,lati,latd,lonp,loni,lond,flagpiloto,setpoint_lat);
        usleep(50000);

    }
    return 0;
}