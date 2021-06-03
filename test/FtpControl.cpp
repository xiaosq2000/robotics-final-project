
#include "FtpControl.h"
#include "Poco/Net/FTPClientSession.h"
#include "Poco/Net/DialogSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Net/NetException.h"
#include <iostream>
#include <string>
#include <fstream>

using namespace std;

bool FtpControl::Upload(std::string ip, std::string workingDirectory, std::string srcfile, std::string dstfile)
{
    try
    {
        Poco::Net::FTPClientSession session(ip);
        if (!session.isOpen())
        {
            return false;
        }

        session.login("", "");
        if (!session.isLoggedIn())
        {
            session.close();
            return false;
        }

        session.setWorkingDirectory(workingDirectory);
        session.setFileType(Poco::Net::FTPClientSession::FileType::TYPE_TEXT);

        fstream fp(srcfile, ios::in);
        if (!fp)
        {
            session.close();
            return false;
        }

        std::istream &ftpin = session.beginList("");
        string list;
        vector<string> filelist;
        while (ftpin >> list)
        {
            if (list == dstfile)
            {
                session.remove(dstfile);
            }
        }
        session.endList();

        ostream& os = session.beginUpload(dstfile);
        string line;
        bool isFirst = true;
        while (getline(fp, line))
        {
            if (line != "")
            {
                if (!isFirst)
                {
                    os << "\r\n" << line;
                }
                else
                {
                    os << line;
                    isFirst = false;
                }
            }
        }
        session.endUpload();

        fp.close();
        session.close();

        return true;
    }
    catch (const Poco::Net::FTPException& e)
    {
        std::cout << e.message() << std::endl;
    }

    return false;
}

bool FtpControl::CheckServer(std::string ip)
{
    try
    {
        Poco::Net::FTPClientSession session(ip);
        if (!session.isOpen())
        {
            return false;
        }

        session.login("", "");
        if (!session.isLoggedIn())
        {
            session.close();
            return false;
        }

        session.close();
        return true;
    }
    catch (const std::exception&)
    {

    }

    return false;
}
