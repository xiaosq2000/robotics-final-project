
/**
 * @file communication.cpp
 * @author 
 * @brief 通过socket完成与机器人的通讯
 * @version 0.1
 * @date 2021-05-18
 * 
 */

#include <iostream>
#include <winsock.h>
#include <conio.h>
using namespace std;

void initialization();
int main()
{
	//定义长度变量
	int send_len = 0;
	int recv_len = 0;
	//定义发送缓冲区和接受缓冲区
	char send_buf[100] = {};
	char recv_buf[200] = {};
	string recvstr;
	//定义服务端套接字，接受请求套接字
	SOCKET s_server;
	//服务端地址客户端地址
	SOCKADDR_IN server_addr;
	initialization();
	//填充服务端信息
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
	server_addr.sin_port = htons(2090);
	//创建套接字
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR)
	{
		cout << "failure: connecting with server" << endl;
		WSACleanup();
	}
	else
	{
		cout << "success: connecting with server" << endl;
	}

	//登录
	send_len = send(s_server, "[1# System.Login 0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);
	//使能
	send_len = send(s_server, "[2# Robot.PowerEnable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	//在此添加程序

	closesocket(s_server);
	//释放DLL资源
	WSACleanup();
	return 0;
}

void initialization()
{
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2); //版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0)
	{
		cout << "failure: initializing socket" << endl;
	}
	else
	{
		cout << "success: initializing socket" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2)
	{
		cout << "failure: socket version mismatching" << endl;
		WSACleanup();
	}
	else
	{
		cout << "success: socket version matching" << endl;
	}
}
