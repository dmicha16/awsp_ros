#include <string>
#include <vector>
#include <wiringPi.h>
#include <wiringSerial.h>


struct position
{
    std::string message;            // The received message
    unsigned long long timestamp;   // Unix time in milliseconds
    float latitude;                 // Always referred to North
    float longitude;                // Always referred to East
    int fix;                        // 0 -> no fix, 1 -> fix, 2 -> dif fix
    int number_of_satelites;        // Satelites on view
    float horizontal_precision;     // In meters;
    float altitude;                 // Over sea level
    float speed;                    // In meters per second
    float true_course;              // True course over ground in degress [0; 360)
};

class GnssInterface
{
    private:
        const std::string PMTK_ACK_START_ = "$PMTK001"; // Acknowledgement

        // UPDATE OUTPUT RATE COMMANDS
        int NMEA_UPDATE_MESSAGE_TYPE = 220;
        const std::string PMTK_SET_NMEA_UPDATE_1HZ_ = "$PMTK220,1000*1F\r\n";
        const std::string PMTK_SET_NMEA_UPDATE_2HZ_ = "$PMTK220,500*2B\r\n";
        const std::string PMTK_SET_NMEA_UPDATE_5HZ_ = "$PMTK220,200*2C\r\n";
        const std::string PMTK_SET_NMEA_UPDATE_10HZ_ = "$PMTK220,100*2F\r\n";

        // BAUD RATE COMMANDS
        const std::string PMTK_SET_BAUD_115200_ = "$PMTK251,115200*1F\r\n";
        const std::string PMTK_SET_BAUD_9600_ = "$PMTK251,9600*17\r\n";

        // NAVIGATION MODE COMMANDS
        const std::string PMTK_FR_MODE_NORMAL_ = "$PMTK886,0*28\r\n";
        const std::string PMTK_FR_FITNESS_NORMAL_ = "$PMTK886,1*29\r\n";

        const std::string POSITION_START_ = "$GPGGA"; // Global positioning system fix data
        const std::string TRACK_START_ = "$GPVTG"; // Track made good and ground speed data
        std::string read_line_;
        int port_;
        position position_;
        std::vector<long> baudrates_{9600, 115200};

        bool position_line_found_ = false;
        bool track_line_found_ = false;

        std::vector<std::string> break_string_(std::string str, char separator);
        unsigned long long get_unix_millis_();
        int parse_acknolegment_response_(std::string response, int sent_command);
        float parse_to_degrees_(std::string str);
        bool parse_raw_line_(std::string line);
        bool populate_position_(std::string position_line);
        bool populate_track_(std::string track_line);
        std::string read_raw_line_();
        void send_command_(std::string command);
    public:
        const int NORMAL_MODE = 0;
        const int FITNESS_MODE = 1;
        GnssInterface();
        ~GnssInterface();
        bool close_connection();
        position get_position();
        int get_port();
        long open_connection(const char* serial_port);
        bool open_connection(const char* serial_port, long baud_rate);
        int read_lines();
        std::vector<std::string> read_raw_lines();
        bool set_baud_rate(long baudrate);
        bool set_fr_mode(int mode);
        bool set_update_rate(int update_rate);
};
