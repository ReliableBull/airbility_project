#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <termio.h>
#include <string.h>


#define BUF_SIZE 59
 
 typedef struct {
    uint16_t timestamp;
    int16_t rollIMUangle;
    int16_t pitchIMUangle;
    int16_t yawIMUangle; 
    int16_t rollIMUspeed;
    int16_t pitchIMUspeed;
    int16_t yawIMUspeed; 
    int16_t rollStatorRotorAngle;
    int16_t pitchStatorRotorAngle;
    int16_t yawStatorRotorAngle; 
} T_GimbalGetAnglesExtReq;


int main(){

    int fd;
    int16_t input;
    unsigned short ooo = 0x05;
    char command[5] = {0x3e,  0x3d , 0x00 , 0x3d , 0x00};

    char buf[BUF_SIZE] = {0x00,};
    //serial connection

    while(1){

    
    fd=open("/dev/ttyUSB1", O_RDWR | O_NOCTTY ); 
     
    assert(fd != -1);

    struct termios newtio;
    // newtio <-- serial port setting.
    memset(&newtio, 0, sizeof(struct termios));
    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag    = IGNPAR ;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = BUF_SIZE;

    
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);



    //std::cout<<strlen(buf);
    
    write(fd, command, 5); // send command
    int res;
    res = read(fd,buf,BUF_SIZE);

    for(int i = 0 ; i < BUF_SIZE ; i++)
    {
        std::cout<<(int)i<<" "<<std::hex<<(short)buf[i]<<std::endl;
        // std::cout<<(int)i<<" "<<(short)buf[i]<<std::endl;
    }    

    std::cout<<std::endl;

    T_GimbalGetAnglesExtReq t;

    int16_t pitch = (int16_t)((buf[25]<<8)) + buf[24];
    int16_t yaw = (int16_t)((buf[43]<<8)) + buf[42];

    t.pitchIMUangle = pitch;
    t.yawIMUangle = yaw;

     std::cout<<"pitch : " <<t.pitchIMUangle*0.0219765625<<std::endl;
    //  std::cout<<"yaw : " <<t.yawIMUangle*0.0219765625<<std::endl;
    std::cout<<"yaw : " <<t.yawIMUangle*(180.0/65536.0) <<std::endl;
    std::cout<<"pitch : " <<t.pitchIMUangle<<std::endl;
    std::cout<<"yaw : " <<t.yawIMUangle<<std::endl;
    
    // while(1)
    // {
    //     if((res = read(fd,buf,BUF_SIZE))>0)
    //     {
    //         for(int i = 0 ; i < BUF_SIZE ; i++)
    //         {
    //             std::cout<<std::hex<<(short)buf[i]<<" ";
    //         }    

    //         break;
    //     }
    // }
    
    
    


      //sleep(2);
      close(fd);
}
}
