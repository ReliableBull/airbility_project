#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <termio.h>
#include <string.h>
#include <cstdint>
#include <cmath> 
#include <iomanip> 

#define BUF_SIZE 47
 
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
// Servo Status 값 계산 함수
// Servo Status 값 계산 함수
// Servo Status 값 계산 함수
double calculateAngle(uint8_t servoStatus) {
    // 8비트 값을 12비트로 확장
    uint16_t expandedValue = (servoStatus) & 0x0F; // 상위 8비트를 왼쪽으로 이동하고 하위 4비트를 0xF로 채움
   std::cout<<std::hex<<expandedValue<<std::endl;
    uint16_t new_yaw = (int16_t)(expandedValue<<8)+ 0xFF;
    std::cout<<std::hex<<new_yaw<<std::endl;
    // 각도 계산
    double angle = (double(new_yaw) * 180.0) / 4095.0;
    std::cout<<angle<<std::endl;
    // 기준값(90도)을 뺌
    angle -= 90.0;

    return angle;
}
int16_t hexToSignedDecimal(uint16_t hexValue) {
    // uint16_t(16비트 무부호 정수)를 int16_t(16비트 부호 있는 정수)로 변환
    return static_cast<int16_t>(hexValue);
}


int main(){

    int fd;
    int16_t input;
    unsigned short ooo = 0x05;
    // char command[5] = {0x3e,  0x3d , 0x00 , 0x3d , 0x00};
    //char command[7] = {0x55,  0xaa , 0xdc , 0x04 , 0x10, 0x00, 0x14};
    //  char startcommand[5] = {0xaa, 0x55, 0x2F, 0x42, 0xFF};
    // char stopcommand[5] = {0xaa, 0x55, 0x2F, 0x02, 0xFF};
    // char EOModeCommand[20] = {0x55, 0xaa, 0xdc, 0x11, 0x30, 0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x81,0x00,0x00,0x00,0xAC};
    // char manualModeCommand[20] = {0x55, 0xaa, 0xdc, 0x11, 0x30, 0x0D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2C};
    char AIDectectionDisableCommand[20] = {0x55, 0xaa, 0xdc, 0x11, 0x30, 0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x2B};
    char WhiteHotIR[20] = {0x55, 0xaa, 0xdc, 0x11, 0x30, 0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x82,0x00,0x00,0x00,0xAF};
    char BlackHotIR[20] = {0x55, 0xaa, 0xdc, 0x11, 0x30, 0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xC2,0x00,0x00,0x00,0xEF};
    char RedHotIR[20] = {0x55, 0xaa, 0xdc, 0x11, 0x30, 0x0F,0x1F,0xFE,0x0E,0x38,0x00,0x00,0x00,0x00,0x04,0x82,0x00,0x00,0x00,0x7F};
    char EOIR[20] = {0x55, 0xaa, 0xdc, 0x11, 0x30, 0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x83,0x00,0x00,0x00,0xAE};


    char buf[BUF_SIZE] = {0x00,};
    //serial connection

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

    std::cout<<strlen(buf); 
    
    write(fd, EOIR, 20); // send command
    // sleep(2.0);
  int k = 0;
    // while(1){
    //   // write(fd, command, 7); 
    // int res;
    // res = read(fd,buf,BUF_SIZE);
    // std::cout<<res<<std::endl;

    // //  std::cout<<buf[1]<<" " <<buf[2]<<" " <<buf[4]<<" " <<std::endl;
    // if(buf[0] == 0x55 && buf[1] == 0xAA && buf[2] == 0xDC && buf[4] == 0x40)
    // {

    
    // // for(int i = 0 ; i < BUF_SIZE ; i++) while(1){
    //   // write(fd, command, 7); 
    // int res;
    // res = read(fd,buf,BUF_SIZE);
    // std::cout<<res<<std::endl;

    // //  std::cout<<buf[1]<<" " <<buf[2]<<" " <<buf[4]<<" " <<std::endl;
    // if(buf[0] == 0x55 && buf[1] == 0xAA && buf[2] == 0xDC && buf[4] == 0x40)
    // {

    
    // // for(int i = 0 ; i < BUF_SIZE ; i++)
    // // {
    // //     std::cout<<(int)i<<" "<<std::hex<<(short)buf[i]<<std::endl;
    // //     // std::cout<<(int)i<<" "<<(short)buf[i]<<std::endl;
    // // }    

    // std::cout<<std::endl;

    // T_GimbalGetAnglesExtReq t;
    // // buf[33] = 0x1B;
    // // buf[34] = 0xF0;
    // // int16_t pitch = (int16_t)(((buf[29]-0xD0)<<8)) + buf[30];
    // // buf[28] = 0xd7;
    // // buf[29] = 0xFF;
    // // int16_t pitch = (int16_t)buf[28];

    // uint16_t roll = (int16_t((buf[28]) & 0x0F)<<8) + buf[29]; 

    // std::cout<<std::hex<<roll<<std::endl;
    // // buf[32] = 0xe3;
    // // buf[33] = 0x8c;
    // int16_t yaw = (int16_t)((buf[32]<<8)) + buf[33];
    
    // int16_t pitch = (int16_t)((buf[30]<<8)) + buf[31];

    
    // // print(pitch*180/4095-90)
    // // double pitch_angle = calculateAngle(pitch);
    // double rollangle = (double(roll) * 180.0) / 4095.0;
    // t.pitchIMUangle = pitch;

    // t.yawIMUangle = hexToSignedDecimal(yaw);
    // // std::cout<<"pitch : " <<(float)pitch*180.0/4095.0 - 90<<std::endl;
    // // std::cout<<"pitch : " <<(int)(pitch*180/4095-90)<<std::endl;
    // //  std::cout<<"pitch : " <<t.pitchIMUangle*0.0219765625<<std::endl;
    
    // std::cout<<"roll : " <<(double)rollangle-90.0<<std::endl;
    // std::cout<<"pitch : " <<(float)t.yawIMUangle*360.0/65536.0<<std::endl;
    // std::cout<<"pitch yaw: " <<(float)pitch * 360/65536<<std::endl;
    // // std::cout<<"yaw : " <<t.yawIMUangle*(180.0/65536.0) <<std::endl;
    // // std::cout<<"pitch : " <<t.pitchIMUangle<<std::endl;
    // // std::cout<<"yaw : " <<t.yawIMUangle<<std::endl;
    

    // //         break;
    // //     }
    // }
    
    
    
    // // {
    // //     std::cout<<(int)i<<" "<<std::hex<<(short)buf[i]<<std::endl;
    // //     // std::cout<<(int)i<<" "<<(short)buf[i]<<std::endl;
    // // }    

    // std::cout<<std::endl;

    // T_GimbalGetAnglesExtReq t;
    // // buf[33] = 0x1B;
    // // buf[34] = 0xF0;
    // // int16_t pitch = (int16_t)(((buf[29]-0xD0)<<8)) + buf[30];
    // // buf[28] = 0xd7;
    // // buf[29] = 0xFF;
    // // int16_t pitch = (int16_t)buf[28];

    // uint16_t roll = (int16_t((buf[28]) & 0x0F)<<8) + buf[29]; 

    // std::cout<<std::hex<<roll<<std::endl;
    // // buf[32] = 0xe3;
    // // buf[33] = 0x8c;
    // int16_t yaw = (int16_t)((buf[32]<<8)) + buf[33];
    
    // int16_t pitch = (int16_t)((buf[30]<<8)) + buf[31];

    
    // // print(pitch*180/4095-90)
    // // double pitch_angle = calculateAngle(pitch);
    // double rollangle = (double(roll) * 180.0) / 4095.0;
    // t.pitchIMUangle = pitch;

    // t.yawIMUangle = hexToSignedDecimal(yaw);
    // // std::cout<<"pitch : " <<(float)pitch*180.0/4095.0 - 90<<std::endl;
    // // std::cout<<"pitch : " <<(int)(pitch*180/4095-90)<<std::endl;
    // //  std::cout<<"pitch : " <<t.pitchIMUangle*0.0219765625<<std::endl;
    
    // std::cout<<"roll : " <<(double)rollangle-90.0<<std::endl;
    // std::cout<<"pitch : " <<(float)t.yawIMUangle*360.0/65536.0<<std::endl;
    // std::cout<<"pitch yaw: " <<(float)pitch * 360/65536<<std::endl;
    // // std::cout<<"yaw : " <<t.yawIMUangle*(180.0/65536.0) <<std::endl;
    // // std::cout<<"pitch : " <<t.pitchIMUangle<<std::endl;
    // // std::cout<<"yaw : " <<t.yawIMUangle<<std::endl;
    
    // // while(1)
    // // {
    // //     if((res = read(fd,buf,BUF_SIZE))>0)
    // //     {
    // //         for(int i = 0 ; i < BUF_SIZE ; i++)
    // //         {
    // //             std::cout<<std::hex<<(short)buf[i]<<" ";
    // //         }    

    // //         break;
    // //     }
    // }
    
    
    


    //   // sleep(0.1);
    
    //   // }
    //   // std::cout<<"122"<<std::endl;
    // }
  close(fd);
}
