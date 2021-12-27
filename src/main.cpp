#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <vector>
#include <mutex>

using namespace std;
using namespace serial;

void serialReadThread();
void run();

Serial s;
mutex m;
bool serialRead = false;
vector<uint8_t> readByte;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm");
    ros::start();
    ros::NodeHandle n;

    s.setPort("/dev/ttyUSB0");
    s.setBaudrate(19200);

    try
    {
        s.open();
    }
    catch (serial::IOException e)
    {
        cerr << "Port open failed." <<e.what()<<endl;
        return false;
    }

    cout<<"port opened"<<endl;

    serialRead = true;
    thread readThread(serialReadThread);
    run();
    serialRead = false;
    readThread.join();
    return 0;
}

// 수신 확인, 수신을 위한 스레드
void serialReadThread()
{
    // 수신한 바이트를 저장할 버퍼
    unsigned char readBuffer[100];

    // 수신 루프
    while(serialRead)
    {
        try // 중간에 통신선이 뽑히거나 할 경우 대비해 예외처리
        {
            int size = s.available();
            if (size != 0)  // 받은 바이트 수가 0이 아닐때
            {
                s.read(readBuffer, size);   // 버퍼에 읽어옴

                m.lock();   // producer consumer pattern. 뮤텍스 lock
                for(int i=0; i<size; i++) readByte.push_back(readBuffer[i]);    // readByte에 하나씩 저장.
                m.unlock(); // 뮤텍스 unlock
            }
        }
        catch (IOException e)   // 예외 발생시 메시지 띄우고 포트 닫는다.
        {
            cerr << "Port disconnected. closing port(" << e.what() << ")." << endl;
            s.close();
        }

        this_thread::sleep_for(chrono::milliseconds(1));    // cpu 점유율 낮추기 위해 잠깐 sleep
    }
}

void run()
{
    // 테스트로 1 2 3을 송신한다.
    vector<uint8_t> packet;
    packet.push_back(0x01);
    packet.push_back(0x02);
    packet.push_back(0x03);
    s.write(packet);

    ros::Rate loopRate(10);
    while (ros::ok())
    {
        //수신 스레드에서 바이트를 수신해 readByte가 차면
        if(!readByte.empty())
        {
            // 한 바이트씩 프린트하고
            cout << "serial received:" << endl;
            for (int i = 0; i < readByte.size(); i++) cout << (int) readByte[i] << endl;
            // readByte를 clear
            readByte.clear();
        }
        ros::spinOnce();
        loopRate.sleep();
    }
}

