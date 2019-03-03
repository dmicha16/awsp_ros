#include <chrono>
#include <iostream>
#include <sstream>
#include <string.h>
#include <thread>
#include "gnss_l86_interface/gnss_l86_lib.h"

// ******************************** CONSTRUCTORS-DESTRUCTORS *******************************
GnssInterface::GnssInterface()
{
    read_line_ = "";
    port_ = 0;
    position_.timestamp = 0;
    position_.latitude = 0;
    position_.longitude = 0;
    position_.fix = 0;
    position_.number_of_satelites = 0;
    position_.horizontal_precision = 0;
    position_.altitude = 0;
}

GnssInterface::~GnssInterface() { }

// **************************************** PRIVATE ****************************************
std::vector<std::string> GnssInterface::break_string_(std::string str, char separator)
{
    std::string result = "";
    std::stringstream ss(str);
    std::string substr;
    std::vector<std::string> content;

    while (ss.good())
    {
        getline(ss, substr, separator);
        if (substr != "")
            content.push_back(substr);
    }

    return content;
}

unsigned long long GnssInterface::get_unix_millis_()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

float GnssInterface::parse_to_degrees_(std::string str)
{
    std::vector<std::string> content = break_string_(str, '.');

    std::string minutes = content[0].substr(content[0].size() - 2);
    minutes += "." + content[1];

    std::string::size_type idx;
    float minutes_float = std::stod(minutes, &idx);
    minutes_float = minutes_float / 60;

    content[0].erase(content[0].end() - 2, content[0].end());
    idx = 0;
    float degrees = std::stod(content[0], &idx) + minutes_float;

    return degrees;
}

bool GnssInterface::parse_raw_line_(std::string line)
{
    if (strncmp(line.c_str(), POSITION_START_.c_str(), POSITION_START_.size()) == 0)
        return populate_position_(line);
    else
        return false;
}

bool GnssInterface::populate_position_(std::string position_line)
{
    std::vector<std::string> content = break_string_(position_line, ',');

    if (content.size() >= 10)
    {
        position_.message = position_line;
        position_.timestamp = get_unix_millis_();

        position_.latitude = parse_to_degrees_(content[2]);
        position_.longitude = parse_to_degrees_(content[4]);
        if (content[3] != "N") position_.latitude = -position_.latitude;
        if (content[5] != "E") position_.longitude = -position_.longitude;

        std::string::size_type idx;
        position_.fix = std::stoi(content[6], &idx);
        idx = 0;
        position_.number_of_satelites = std::stoi(content[7], &idx);
        idx = 0;
        position_.horizontal_precision = std::stof(content[8], &idx);
        idx = 0;
        position_.altitude = std::stof(content[9], &idx);

        return true;
    }
    else
    {
        position_.fix = 0;
        return false;
    }
}

void GnssInterface::send_command_(std::string command)
{
    for (int i = 0; i < command.size(); ++i) serialPutchar(port_, command[i]);
}

// **************************************** PUBLIC *****************************************
bool GnssInterface::close_connection()
{
    serialClose(port_);
    return true;
}

int GnssInterface::get_port()
{
    return port_;
}

position GnssInterface::get_position()
{
    return position_;
}

long GnssInterface::open_connection(const char* serial_port)
{
    for (int i = 0; i < baudrates_.size(); ++i)
    {
        if (open_connection(serial_port, baudrates_[i]))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            serialFlush(port_);
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            std::vector<std::string> lines = read_raw_lines();

            if (lines.size() > 2)
            {
                std::string line = lines[2];
                std::string start = "$";
                if (strncmp(line.c_str(), start.c_str(), start.size()) == 0)
                    return baudrates_[i];
            }
            close_connection();
        }
    }

    return 0;
}

bool GnssInterface::open_connection(const char* serial_port, long baud_rate)
{
    port_ = serialOpen(serial_port, baud_rate);

    if (port_ >= 0)
    {
        read_raw_lines();
        return true;
    }
    else return false;
}

int GnssInterface::read_lines()
{
    std::vector<std::string> raw_lines = read_raw_lines();
    int num_lines = 0;

    for (int i = 0; i < int(raw_lines.size()); i++)
        if (parse_raw_line_(raw_lines[i])) num_lines++;

    return num_lines;
}

std::vector<std::string> GnssInterface::read_raw_lines()
{
    char received;
    std::vector<std::string> lines;
    while (serialDataAvail(port_))
    {
        received = serialGetchar(port_);
        if (received != '\n')
            read_line_ += received;
        else
        {
            lines.push_back(read_line_);
            read_line_ = "";
        }
    }

    return lines;
}

bool GnssInterface::set_baud_rate(long baudrate)
{
    if (baudrate == 9600) send_command_(PMTK_SET_BAUD_9600_);
    else if (baudrate == 115200) send_command_(PMTK_SET_BAUD_115200_);
    else return false;
    
    return true;
}
