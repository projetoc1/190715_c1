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

//setup de servos

RCOutput_Navio2 servo1;

#define SERVO_MIN 1000 /*mS*/
#define SERVO_MAX 2000 /*mS*/
#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1
#define PWM_OUTPUT_3 2
#define PWM_OUTPUT_4 3

float comando_1 = 1000;
float comando_2 = 1000;
float comando_3 = 1000;
float comando_4 = 1000;


//INERCIAIS

//MPU9250

MPU9250 sensormpu;

float axmpu, aympu, azmpu;
float gxmpu, gympu, gzmpu;
float mxmpu, mympu, mzmpu;

//LSM9DS1

LSM9DS1 sensorlsm;

float axlsm, aylsm, azlsm;
float gxlsm, gylsm, gzlsm;
float mxlsm, mylsm, mzlsm;

//CONVERSORES ADC

ADC_Navio2 ADC;

float ch0 = 0;
float ch1 = 0;
float ch2 = 0;
float ch3 = 0;
float ch4 = 0; //CANAIS DA PORTA ADC NA NAVIO2 
float ch5 = 0; //CANAIS DA PORTA ADC NA NAVIO2

float pot1=0;

//VARIÁVÉIS DA TELA NEXTION

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
float latnavio2 = 150;
float lonnavio2 = 50;
float leituraadc1 = 0;
float leituraadc2 = 0;

// Variáveis para medição de tempo

struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;

//MUTEX

pthread_mutex_t trava;

//FUNÇÃO MONTADA EM FORMA DE THREAD DA TELA NEXTION

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

//variaveis do programa principal
    float ed = (93.066*pot1) - 28.598 - 80;
    float xd = 0, xd_ponto, c = 0.2, wd = 0 ;
    float intervalo = 20;
    float teta_integrado = 0;

    float P=0,I=0,D=0,PID=0;
    
    float setpoint; //graus
    float setpoint0 = 15; //graus
    float kp   = -0.05; //para massa maior --> -0.013
    float ki   = 0.05; //para massa maior --> 0.01
    float kd   = 0.0;
    float k_teta_ponto = 0.0;
    float teta_p = 0;
    float xtp = 0;
    float xtpp=0;
    float ctp = 0.1;
    float e = 0; 
    float teta = 0; 
    float xi = 0;
    float wi = 0;
    float u = 0;
    float pmx = 1408;
    float pmi = 1192;
    float T = 0.5;
    float up,ui,ud,utp;

//PROGRAMA PRINCIPAL

int main()
{   
    //TESTA FUNCIONAMENTO DA NAVIO2
    pthread_mutex_t trava;
    if (check_apm()) 
    {
        return 1;
    }

    //THREADS//

    pthread_t tela;
    pthread_create(&tela,NULL,f_tela,NULL);

    //INICIALIZA ADC's , inerciais, SERVOS e tela NEXTION
    nexInit();
    ADC.initialize();
    sensormpu.initialize();
    sensorlsm.initialize();

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

    //LEITURAS INICIAIS
    ch5 = ADC.read(4);
    pot1 = ch5/1000;

    teta = (93.066*pot1) - 28.598 - 80;
    float tetaf = teta;

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

        ch5 = ADC.read(4); //ADC3 DA PORTA ADC NA NAVIO2
        pot1 = ch5/1000;
        teta = (93.066*pot1) - 28.598 - 80;
        xtpp  = (-1/ctp)*xtp +teta;
        xtp=xtp+xtpp*dt;
        teta_p=(-1/(ctp*ctp))*xtp + (1/ctp)*teta;

        tetaf = tetaf + (-tetaf/T + teta/T)*dt;
        e = setpoint - teta;
        xi = xi + e*dt;
        wi = xi;

        xd_ponto = (-1/c)*xd + e;
        xd       = xd + xd_ponto*dt;
        wd       = (-1/(c*c))*xd + (1/c)*e;

        if(segundos>0.3)
        {
        	u = ki*wi + kd*wd + kp*e + k_teta_ponto*teta_p;

		}
		else
		{
			u = ki*wi + kp*e;
		}
		up= kp*e;
		ui=ki*wi;
		ud=kd*wd;
		utp= k_teta_ponto*teta_p;

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

        if(u>1.2)
        	{u=1.2;}


        if(u<-1.2)
        	{u=-1.2;}


        servo1.set_duty_cycle(PWM_OUTPUT_1, 1000);
        servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);

        servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
        //servo1.set_duty_cycle(PWM_OUTPUT_4, comando_3); //para massa maior --> pasar esse comando e comentar a linha abaixo
        servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);

        //printf("TETA = %.2f TETAF %.2f erro = %.2f TETA_P = %.2f PWM1 = %.2f PWM2 = %.2f up= %.2f ui= %.2f ud= %.2f utp= %.2f\n",teta,tetaf,e,teta_p,comando_1,comando_3,up,ui,ud,utp);
        printf("kp = %.2f kp tela = %.2u \n",kp,latp);
        usleep(20000);

    }
    return 0;
}