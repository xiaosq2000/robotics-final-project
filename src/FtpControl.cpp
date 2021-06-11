
#include <iostream>
#include <string>
#include <fstream>

#include "Poco/Net/FTPClientSession.h"
#include "Poco/Net/DialogSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Net/NetException.h"

#include "FtpControl.h"

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

        std::fstream fp(srcfile, std::ios::in);
        if (!fp)
        {
            session.close();
            return false;
        }

        std::istream &ftpin = session.beginList("");
        std::string list;
        std::vector<std::string> filelist;
        while (ftpin >> list)
        {
            if (list == dstfile)
            {
                session.remove(dstfile);
            }
        }
        session.endList();

        std::ostream &os = session.beginUpload(dstfile);
        std::string line;
        bool isFirst = true;
        while (std::getline(fp, line))
        {
            if (line != "")
            {
                if (!isFirst)
                {
                    os << "\r\n"
                       << line;
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
    catch (const Poco::Net::FTPException &e)
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
    catch (const std::exception &)
    {
    }

    return false;
}
