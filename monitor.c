#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>

#define BME280_S32_t long signed int
#define BME280_U32_t long unsigned int
#define BME280_S64_t long long signed int

#define BME280_ADDRESS 0x76
#define ADS1015_ADDRESS 0x48


//#define SPI_BITS 8
//#define SPI_DEVICE1 "/dev/spidev0.0"
//#define SPI_DEVICE2 "/dev/spidev0.1"

void main(void);

//I2C settings
int setup_BME280();
int setupi2c();
int setupi2cdev(int fd,int dev);
int writei2c(int fd,unsigned char buf[],int databyte);
int readi2c(int fd,unsigned char buf[],int databyte);
int writei2c8bits(int fd,unsigned char addr,unsigned char data);
unsigned char  readi2c8bits(int fd,unsigned char data);
int writei2c16bits(int fd,unsigned char addr,unsigned short int data);
unsigned short  readi2c16bits(int fd,unsigned char addr);

//ADS1015 settings
int setup_ADS1015();

#define  conv_reg  0b00000000
#define  conf_reg  0b00000001
#define  lo_th_reg 0b00000010
#define  hi_th_reg 0b00000011
//get ADS1015
unsigned short get_ADS1015(int fd,char mux,char pga);



//get daytime
void get_daytime (char Time_string[]);

// get cputemp
float get_cputemp(void);

//BME280 settings

void readTrim(int fd);
void readRawData(int fd);
BME280_S32_t cali_T(BME280_S32_t adc_T);
BME280_U32_t cali_P(BME280_S32_t adc_P);
BME280_U32_t cali_H(BME280_S32_t adc_H);

BME280_S32_t hum_raw,temp_raw,pres_raw;
BME280_S32_t t_fine;

 unsigned short dig_T1;
 signed short dig_T2;
 signed short dig_T3;

 unsigned short dig_P1;
 signed short dig_P2;
 signed short dig_P3;
 signed short dig_P4;
 signed short dig_P5;
 signed short dig_P6;
 signed short dig_P7;
 signed short dig_P8;
 signed short dig_P9;

 unsigned char dig_H1;
 signed short dig_H2;
 unsigned char dig_H3;
 signed short dig_H4;
 signed short dig_H5;
 signed char  dig_H6;

//main routine

void main(void)
{
      int fd_spi,fd_i2c;
         int value,i;
    float volt[3];
 float cpu_temp;

  FILE *fp;
double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
   int ret;
    BME280_S32_t temp_cal;
    BME280_U32_t press_cal,hum_cal;

char DayTime[128];
char out_char[256];

//Get Daytime
   get_daytime (DayTime);

//Get Temp. Hum. Press
    fd_i2c=setup_BME280();
      readTrim(fd_i2c);
        readRawData(fd_i2c);
      close(fd_i2c);

    temp_cal = cali_T(temp_raw);
    press_cal = cali_P(pres_raw);
    hum_cal = cali_H(hum_raw);

    temp_act = (double)temp_cal / 100.0;
    press_act = (double)press_cal / (256*100);
    hum_act = (double)hum_cal / 1024.0;

   fd_i2c=setup_ADS1015();
 volt[0]=0.5/1000.0*((float) get_ADS1015(fd_i2c,4,2))-0.6;
 volt[1]=6.144*((float) get_ADS1015(fd_i2c,5,0)/4096.0);
     close(fd_i2c);

// Get cputemp.
cpu_temp=get_cputemp();


    // make output datas
   sprintf(out_char,"%s %f %f %f %f %f %f \n",DayTime,temp_act,hum_act,press_act,cpu_temp,100.0*volt[0],volt[1]);
        printf("%s",out_char);
}

//ADS1015 ROUTINE

int setup_ADS1015()
{

//8,583 //ctrl cmd
// 1000 0101 1000 0011
    unsigned char os  =          0b01;
    unsigned char mux =        0b0000;
    unsigned char pga =        0b0000;
    unsigned char mode =         0b01;
    unsigned char dr =         0b0111;
    unsigned char comp_mode=   0b0011;

    unsigned int ctrl1 =(os << 15) | (mux << 12) | (pga << 9) | (mode << 8) | (dr << 5) | comp_mode;

   int fd;
   int ret;


  fd=setupi2c();
     if(fd < 0) exit(-1);
      ret=setupi2cdev(fd,ADS1015_ADDRESS);
    ret = writei2c16bits(fd,conf_reg,ctrl1);

   return(fd);
}

unsigned short get_ADS1015(int fd,char mux,char pga)
{

 unsigned int ctrl1 = (mux << 12) | (pga << 9) ;
 unsigned short ret;
int ret2;
     ret=  readi2c16bits(fd,conf_reg);
    ret2 = writei2c16bits(fd,conf_reg, ret | ctrl1);

       return(readi2c16bits(fd,conv_reg)>>3);
}






// BME280 ROUTINE

int setup_BME280()
{
    unsigned char osrs_t =  0b001;             //Temperature oversampling x 1
    unsigned char osrs_p =  0b001;             //Pressure oversampling x 1
    unsigned char osrs_h =  0b001;             //Humidity oversampling x 1
    unsigned char mode =    0b01;              //Forced mode
    unsigned char t_sb =    0b101;             //Tstandby 1000ms
    unsigned char filter =  0b000;             //Filter off
    unsigned char spi3w_en =0b0;               //3-wire SPI Disable

    unsigned int ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    unsigned int config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    unsigned int ctrl_hum_reg  = osrs_h;

     int ret;
      int fd;

        fd = setupi2c();
     if (fd <0 )  exit(-1);
          ret =setupi2cdev(fd,BME280_ADDRESS);
                if (fd <0 ) { printf("Cannot find %0X\n",BME280_ADDRESS);
                                           exit(-1);}
  // write to configretion reg.

    ret= writei2c8bits(fd,0xf4,ctrl_meas_reg);

    ret= writei2c8bits(fd,0xf5,config_reg);

    ret= writei2c8bits(fd,0xf2,ctrl_hum_reg);

      return(fd);
}

 int  setupi2c()
{
      int fd;
      fd=open("/dev/i2c-1",O_RDWR);
       if (fd < 0) {printf("Cannot open i2c-1 device\n");
        return(-1);
      }
      return(fd);
}

    int  setupi2cdev(int fd,int addr)
{      int ret;
      ret=ioctl(fd,I2C_SLAVE,addr);
  return(ret);
}

      int  writei2c(int fd,unsigned char buf[],int databyte)
{
       int ret;
          ret=write(fd,buf,databyte);
     if ( ret != databyte ) {
    printf("write error /n");
       return (-1);
}
     return(ret);
}
 
     int readi2c(int fd,unsigned char buf[],int databyte)
{
     int  ret;
      ret=read (fd,buf,databyte);
         if (ret != databyte){
     printf("read error!/n");
       return(-1);
}

       return(ret);
}

    int writei2c8bits(int fd,unsigned char addr,unsigned char data)
{
   int ret;
    unsigned char  buf[2];
       buf[0]=addr;
       buf[1]=data;
       buf[2]=0;
  ret=writei2c(fd,buf,2);
     return(ret);
}

 unsigned char  readi2c8bits(int fd,unsigned char data)
{
   int ret;
    unsigned char  buf[1];
      buf[0]=data;
     buf[1]=0;
     writei2c(fd,buf,1);
       readi2c(fd,buf,1);
return(buf[0]);
}



void readTrim(int fd)
{
   char data[32],i,j;

j=0;
  for (i=0x88;i<=0xa1;i++){data[j]=0;
//    data[j] =wiringPiI2CReadReg8(fd,i);
    data[j]= readi2c8bits(fd,i);
     j++;
   }

  for (i=0xE1;i<=0xe7;i++){data[j]=0;
//    data[j] =wiringPiI2CReadReg8(fd,i)
    data[j]= readi2c8bits(fd,i);
   j++;
   }
 
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];

    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];

    dig_H1 = data[25];
    dig_H2 = (data[27]<< 8) | data[26];
    dig_H3 = data[28];
    dig_H4 = (data[29]<< 4) | (data[30] & 0x0F);
    dig_H5 = (data[31] << 4) | ((data[30] >> 4) & 0x0F); 
    dig_H6 = data[32];  
}

void readRawData(int fd)
{
    int i=0,j=0;
    unsigned char data[8];
           for (i=0xf7;i<=0xfe;i++){
 //               data[j]=wiringPiI2CReadReg8(fd,i);
                 data[j]= readi2c8bits(fd,i);
  j++;
}

    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    hum_raw  = (data[6] << 8) | data[7];
}




BME280_S32_t cali_T(BME280_S32_t adc_T)
{

    BME280_S32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((BME280_S32_t)dig_T1 << 1))) * ((BME280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((BME280_S32_t)dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)dig_T1))) >> 12) *
    ((BME280_S32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}

BME280_U32_t cali_P(BME280_S32_t adc_P)
{
    BME280_S64_t var1, var2,p;
    
    var1 = ((BME280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)dig_P6;
    var2 = var2 + ((var1*(BME280_S64_t)dig_P5)<<17);
    var2 = var2 + (((BME280_S64_t)dig_P4)<<35);
    var1 = (( var1 * var1* (BME280_S64_t)dig_P3)>>8) + (var1* (BME280_S64_t)dig_P2<<12);
    var1 = (((((BME280_S64_t)1)<<47)+var1))*((BME280_S64_t)dig_P1)>>33;
    if (var1 == 0)
    {
        return 0;
    }    
    p = 1048576-adc_P;
         p=(((p<<31)-var2)*3125)/var1;
             var1=(((BME280_S64_t)dig_P9)*(p>>13)*(p>>13)) >>25;
                                var2=((BME280_S64_t)dig_P8 *p) >> 19;
            p=((p+var1+var2)>>8)+(((BME280_S64_t)dig_P7)<<4);
       return (BME280_U32_t)p;
      }
  
BME280_U32_t cali_H(BME280_S32_t adc_H)
{
    BME280_S32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20)-(((BME280_S32_t)dig_H5) * v_x1_u32r)) + 
        ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r *
        ((BME280_S32_t)dig_H3)) >> 11)+((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) * 
        ((BME280_S32_t)dig_H2)+8192) >> 14));


         v_x1_u32r=(v_x1_u32r-(((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)*((BME280_S32_t)dig_H1)) >> 4));
 
                   v_x1_u32r=(v_x1_u32r < 0 ? 0 : v_x1_u32r);
                              v_x1_u32r =(v_x1_u32r > 419430400 ? 41930400 : v_x1_u32r);

   return (BME280_U32_t)(v_x1_u32r >> 12);   
}


void get_daytime (char Time_string[])
{
    time_t now;
    struct tm *tm;
    now = time(0);
    if ((tm = localtime (&now)) == NULL) {
        printf ("Error extracting time stuff\n");
        return ;
    }

//    sprintf (Time_string,"%04d-%02d-%02d %02d:%02d:%02d",
  //      tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
    //    tm->tm_hour, tm->tm_min, tm->tm_sec);


    sprintf (Time_string,"%02d:%02d",tm->tm_hour,tm->tm_min);
       return;
}

 float get_cputemp(void) { FILE *fd;
      int temp;
      float temp_act;
     fd=fopen ("/sys/class/thermal/thermal_zone0/temp","r");
       fscanf(fd,"%d",&temp);
          temp_act=(float)temp/1000.0;
    fclose(fd);
  return(temp_act);
}

 int writei2c16bits(int fd,unsigned char addr,unsigned short int data)
{
   int ret;
    unsigned char  buf[2];
       buf[0]=addr;
         buf[1]=data >>8;
     buf[2]=data & 0x00ff;
   buf[3]=0;
  ret=   writei2c(fd,buf,3);

     return(ret);
}

 unsigned short  readi2c16bits(int fd,unsigned char addr)
{
   int ret;
    unsigned char  buf[1];
      buf[0]=addr;
   buf[1]=0;
     writei2c(fd,buf,1);
       readi2c(fd,buf,2);
return( buf[0] << 8 | buf[1]);
}
