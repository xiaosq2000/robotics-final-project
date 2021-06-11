
#ifndef _SAMPLE_H_
#define _SAMPLE_H_

#include <iostream>
#include <string>

class Sample
{
private:
    std::string img_directory_;
    std::string rpy_file_path_;
public:
    Sample(std::string img_directory, std::string rpy_file_path);
    ~Sample();
};

#endif
