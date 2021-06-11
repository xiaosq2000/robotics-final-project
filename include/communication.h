
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include <iostream>
#include <string>
#include <winsock.h>
#include <conio.h>

class Communication
{
private:
    const char *internet_address_;
    int port_;
    SOCKET s_server_;
    int send_len_;
    int recv_len_;
    char send_buf_[128];
    char recv_buf_[128];

public:
    Communication(const char *internet_address = "192.168.10.120", const int &port = 2090);
    void SendInstruction(const std::string &str);
    ~Communication();
};

#endif
