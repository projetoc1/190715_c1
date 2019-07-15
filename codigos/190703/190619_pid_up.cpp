//BIBLIOTECAS
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

//DEFINIÇÕES PARA PORTAS DE PWM
#define SERVO_MIN 1000 /*mS*/
#define SERVO_MAX 2000 /*mS*/
#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1
#define PWM_OUTPUT_3 2
#define PWM_OUTPUT_4 3

//MUTEX
pthread_mutex_t trava;

//LEITURA ADC
ADC_Navio2 ADC;

float ch4  = 0; //CANAIS DA PORTA ADC NA NAVIO2 
float ch5  = 0; //CANAIS DA PORTA ADC NA NAVIO2
float pot1 = 0;
float pot2 = 0;

//PWM 
RCOutput_Navio2 servo1;

float comando_1 = 1000;
float comando_2 = 1000;
float comando_3 = 1000;
float comando_4 = 1000;

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;

//TELA NEXTION 
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
uint32_t        latp  = 0;
uint32_t        lati  = 0;
uint32_t        latd  = 0;
uint32_t        lonp  = 0;
uint32_t        loni  = 0;
uint32_t        lond  = 0;
float        errorlat = 3;
float        errorlon = 4;
uint32_t        flag0 = 0;
uint32_t   flagpiloto = 0;
float       latnavio2 = 9;
float       lonnavio2 = 5;
float     leituraadc1 = 6;
float     leituraadc2 = 7;

//FUNÇÃO THREAD TELA NEXTION
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
	//INICIALIZA TELA NEXTION
    nexInit();

    //CHECA NAVIO2
    if (check_apm()) 
    {
        return 1;
    }

    //MULTITHREADING - TELA
    pthread_t tela;
    pthread_create(&tela,NULL,f_tela,NULL);

    //HABILITA LEITURA DE PORTAS ADC's
    ADC.initialize();

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

    //MOTORES DESLIGADOS
    servo1.set_duty_cycle(PWM_OUTPUT_1, 1000);
    servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
    servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);
    servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);

    //LEITURA INICIAL DE DOS POTENCIOMETROS
    ch5  = ADC.read(4);
    ch4  = ADC.read(5);
    pot1 = ch5/1000;
    pot2 = ch4/1000;

    float setpoint0  = 10; //graus
    float setpoint;       //graus

    float setpoint0z = 10; //graus
    float setpointz;       //graus

    //EIXO Y
    float teta_p = 0;
    float teta   = 0; // EIXO Y
    float xd     = 0;
    float c      = 0.2;
    float wd     = 0;
    float kp     = -0.05; //para massa maior --> -0.013
    float ki     = 0.1;   //para massa maior --> 0.01
    float kd     = 0.0;
    float xtp    = 0;
    float xtpp   = 0;
    float ctp    = 0.1;
    float e      = 0; 
    float xi     = 0;
    float wi     = 0;
    float T      = 0.5;
    float pmx    = 1408; // máximo de comando do motor
    float pmi    = 1192; // mínimo de comando do motor
    float u      = 0;
    float up;
    float ui;
    float ud;
    float utp;
    float xd_ponto;
    float k_teta_ponto = 0.0;
    teta = (93.066*pot1) - 28.598 - 80;
    float tetaf = teta;

        //EIXO Z
    float teta_pz = 0;
    float tetaz   = 0; // EIXO Y
    float xdz     = 0;
    float cz      = 0.2;
    float wdz     = 0;
    float kpz     = -0.05; //para massa maior --> -0.013
    float kiz     = 0.1;   //para massa maior --> 0.01
    float kdz     = 0.0;
    float xtpz    = 0;
    float xtppz   = 0;
    float ctpz    = 0.1;
    float ez      = 0; 
    float xiz     = 0;
    float wiz     = 0;
    float Tz      = 0.5;
    float pmxz    = 1308; // máximo de comando do motor
    float pmiz    = 1192; // mínimo de comando do motor
    float uz      = 0;
    float upz;
    float uiz;
    float udz;
    float utpz;
    float xd_pontoz;
    float k_teta_pontoz = 0.0;
    tetaz = (93.066*pot2) - 28.598 - 80;
    float tetafz = tetaz;

    while(true)
    {
    	//advindos do display NEXTION
        kp  = (0.4/100)*(float)latp - 0.2;
        ki  = (0.4/100)*(float)lati - 0.2;
        kd  = (0.4/100)*(float)latd - 0.2;

        kpz = (0.4/100)*(float)lonp - 0.2;
        kiz = (0.4/100)*(float)loni - 0.2;
        kdz = (0.4/100)*(float)lond - 0.2;

        //PEGA SETPOINT DA TELA CASO DIFERENTE DE ZERO
        //EIXO Y
        if (setpoint_lat!=300)
        {
        	setpoint  = float(setpoint_lat) - 300;
        }
        setpoint = setpoint0*(1-exp(-segundos/1));
        //EIXO Z
        if (setpoint_lon!=300)
        {
        	setpointz = float(setpoint_lon) - 300;
        }
        setpointz = setpoint0z*(1-exp(-segundos/1));

        //Contagem de tempo. Tempo total : variável segundos
        //                   Delta       : varíavel dt     
        gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        if (dt>100)
        {
            dt = 0;
        }

        segundos = segundos + dt;

        //Leitura do canal ADC
        ch4 = ADC.read(5); //eixo Z
        ch5 = ADC.read(4); //eixo Y

        //Converte para tensão
        pot1 = ch5/1000;
        pot2 = ch4/1000;

        //Converte para ângulo
        teta  = (93.066*pot1) - 28.598 - 80;
        tetaz = (93.066*pot2) - 28.598 - 80;

        //Envia para tela campo leitura ADC
        leituraadc1 = teta + 300;
        leituraadc2 = teta + 300;

        //ROTINA CONTROLADOR PID EIXO Y
        xtpp   = (-1/ctp)*xtp +teta;
        xtp    = xtp + xtpp*dt;
        teta_p = (-1/(ctp*ctp))*xtp + (1/ctp)*teta;

        tetaf    = tetaf + (-tetaf/T + teta/T)*dt;
        e        = setpoint - teta;
        errorlat = e; //enviando para tela o erro no eixo Y
        xi       = xi + e*dt;
        wi       = xi;

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
		up  = kp*e;
		ui  = ki*wi;
		ud  = kd*wd;
		utp = k_teta_ponto*teta_p;

        //LIMITADORES:
        if(u>1)
        {
        	u=1;
        }
        if(u<-1)
        {
        	u=-1;
        }


        comando_1 = (pmx+pmi)/2  + (pmx-pmi)*u/2;
        comando_3 = (pmx+pmi)/2  - (pmx-pmi)*u/2;

        //ROTINA CONTROLADOR PID EIXO Z
        xtppz   = (-1/ctpz)*xtpz +tetaz;
        xtpz    = xtpz+xtppz*dt;
        teta_pz = (-1/(ctpz*ctpz))*xtpz + (1/ctpz)*tetaz;

        tetafz = tetafz + (-tetafz/Tz + tetaz/Tz)*dt;
        ez = setpoint - tetaz;
        errorlon = ez; //enviando para tela erro no eixo Z
        xiz = xiz + ez*dt;
        wiz = xiz;

        xd_pontoz = (-1/cz)*xdz + ez;
        xdz       = xdz + xd_pontoz*dt;
        wdz       = (-1/(cz*cz))*xdz + (1/cz)*ez;

        if(segundos>0.3)
        {
        	uz = kiz*wiz + kdz*wdz + kpz*ez + k_teta_pontoz*teta_pz;

		}
		else
		{
			uz = kiz*wiz + kpz*ez;
		}
		upz  = kpz*ez;
		uiz  = kiz*wiz;
		udz  = kdz*wdz;
		utpz = k_teta_pontoz*teta_pz;

        //LIMITADORES:
		if(uz>1)
        {
        	uz=1;
        }
        if(uz<-1)
        {
        	uz=-1;
        }

        comando_2 = (pmxz+pmiz)/2  + (pmxz-pmiz)*uz/2;
        comando_4 = (pmxz+pmiz)/2  - (pmxz-pmiz)*uz/2;

        //EIXO Z
        servo1.set_duty_cycle(PWM_OUTPUT_1, 1000);
        servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);

        //EIXO Y
        servo1.set_duty_cycle(PWM_OUTPUT_2, comando_1);
        servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);

		usleep(20000);

    }
    return 0;
}