
#include <iostream>
#include <winsock.h>
#include <conio.h>
#include "communication.h"

Communication::Communication(const char *internet_address, const int &port)
{
    this->internet_address_ = internet_address;
    this->port_ = port;

    WORD w_req = MAKEWORD(2, 2);
    WSADATA wsadata;
    int err;
    err = WSAStartup(w_req, &wsadata);
    if (err != 0)
    {
        std::cout << "failure: initializing socket" << std::endl;
    }
    else
    {
        std::cout << "success: initializing socket" << std::endl;
    }
    if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2)
    {
        std::cout << "failure: socket version mismatching" << std::endl;
        WSACleanup();
    }
    else
    {
        std::cout << "success: socket version matching" << std::endl;
    }

    SOCKADDR_IN server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.S_un.S_addr = inet_addr(this->internet_address_);
    server_addr.sin_port = htons(this->port_);
    this->s_server_ = socket(AF_INET, SOCK_STREAM, 0);
    if (connect(this->s_server_, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR)
    {
        std::cout << "failure: connecting with the server" << std::endl;
        WSACleanup();
    }
    else
    {
        std::cout << "success: connecting with the server" << std::endl;
    }

    this->SendInstruction("[0# System.Login 0]");
    this->SendInstruction("[0# Robot.PowerEnable 1,1]");
    this->SendInstruction("[0# System.Abort 1]");
    this->SendInstruction("[0# System.Start 1]");
    this->SendInstruction("[0# Robot.Home 1]");
}

Communication::~Communication()
{
    this->SendInstruction("[0# Robot.Home 1]");
    this->SendInstruction("[0# System.Abort]");
    this->SendInstruction("[0# System.Logout]");
    closesocket(this->s_server_);
    WSACleanup();
}

void Communication::SendInstruction(const std::string &str)
{
    this->send_len_ = send(this->s_server_, str.c_str(), 128, 0);
    this->recv_len_ = recv(this->s_server_, this->recv_buf_, 128, 0);
    std::cout << this->recv_buf_ << std::endl;
    memset(this->recv_buf_, '\0', sizeof(this->recv_buf_));
    Sleep(500);
}
