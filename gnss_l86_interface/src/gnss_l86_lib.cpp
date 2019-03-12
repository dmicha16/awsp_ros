#include <chrono>
#include <iostream>
#include <sstream>
#include <string.h>
#include <thread>
#include <errno.h>
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

int GnssInterface::parse_acknolegment_response_(std::string response, int sent_command)
{
    if (strncmp(response.c_str(), PMTK_ACK_START_.c_str(), PMTK_ACK_START_.size()) == 0)
    {
        std::vector<std::string> content = break_string_(response, ',');
        if (std::stoi(content[1]) != sent_command) return -1;
        else return std::stoi(content[2]);
    }
    else return -2;
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
    {
        position_line_found_ = true;
        return populate_position_(line);
    }
    else if (strncmp(line.c_str(), TRACK_START_.c_str(), TRACK_START_.size()) == 0)
    {
        track_line_found_ = true;
        return populate_track_(line);
    }
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

bool GnssInterface::populate_track_(std::string track_line)
{
    std::vector<std::string> content = break_string_(track_line, ',');

    if (content.size() >= 8)
    {
        position_.true_course = std::stof(content[1]);
        position_.speed = std::stof(content[6]);
        return true;
    }
    else
    {
        position_.fix = 0;
        return false;
    }
}

std::string GnssInterface::read_raw_line_()
{
    char received;
    std::string line;
    while (serialDataAvail(port_))
    {
        received = serialGetchar(port_);
        if (received != '\n')
            line += received;
        else break;
    }

    return line;
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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            serialFlush(port_);
            std::this_thread::sleep_for(std::chrono::milliseconds(1100));
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

    for (int i = int(raw_lines.size()) - 1; i >= 0; i--)
    {
        if (parse_raw_line_(raw_lines[i])) num_lines++;
        if (position_line_found_ && track_line_found_)
        {
            position_line_found_ = false;
            track_line_found_ = false;
            break;
        }
    }

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

bool GnssInterface::set_fr_mode(int mode)
{
    if (mode == NORMAL_MODE) send_command_(PMTK_FR_MODE_NORMAL_);
    else if (mode == FITNESS_MODE) send_command_(PMTK_FR_FITNESS_NORMAL_);
    else return false;

    return true;
}

bool GnssInterface::set_update_rate(int update_rate)
{
    if (update_rate == 1) send_command_(PMTK_SET_NMEA_UPDATE_1HZ_);
    else if (update_rate == 2) send_command_(PMTK_SET_NMEA_UPDATE_2HZ_);
    else if (update_rate == 5) send_command_(PMTK_SET_NMEA_UPDATE_5HZ_);
    else if (update_rate == 10) send_command_(PMTK_SET_NMEA_UPDATE_10HZ_);
    else return false;

    return true;
}
