#include <string>
#include <vector>
#include <wiringPi.h>
#include <wiringSerial.h>


struct position
{
    std::string message;            // The received message
    unsigned long long timestamp;   // Unix time in milliseconds
    float latitude;                // Always referred to North
    float longitude;               // Always referred to East
    int fix;                        // 0 -> no fix, 1 -> fix, 2 -> dif fix
    int number_of_satelites;        // Satelites on view
    float horizontal_precision;     // In meters;
    float altitude;                 // Over sea level
};

class GnssInterface
{
    private:
        // UPDATE RATE COMMANDS
        const std::string PMTK_SET_NMEA_UPDATE_1HZ_ = "$PMTK220,1000*1F\r\n";
        const std::string PMTK_SET_NMEA_UPDATE_2HZ_ = "$PMTK220,500*2B\r\n";
        const std::string PMTK_SET_NMEA_UPDATE_5HZ_ = "$PMTK220,200*2C\r\n";
        const std::string PMTK_SET_NMEA_UPDATE_10HZ_ = "$PMTK220,100*2F\r\n";
        // FIX CLOCK COMMANDS
        const std::string PMTK_API_SET_FIX_CTL_1HZ_ = "$PMTK300,1000,0,0,0,0*1C\r\n";
        const std::string PMTK_API_SET_FIX_CTL_5HZ_ = "$PMTK300,200,0,0,0,0*2F\r\n";
        // BAUD RATE COMMANDS
        const std::string PMTK_SET_BAUD_115200_ = "$PMTK251,115200*1F\r\n";
        const std::string PMTK_SET_BAUD_9600_ = "$PMTK251,9600*17\r\n";

        const std::string POSITION_START_ = "$GPGGA";
        std::string read_line_;
        int port_;
        position position_;
        std::vector<long> baudrates_{9600, 115200};

        std::vector<std::string> break_string_(std::string str, char separator);
        unsigned long long get_unix_millis_();
        float parse_to_degrees_(std::string str);
        bool parse_raw_line_(std::string line);
        bool populate_position_(std::string position_line);
        void send_command_(std::string command);
    public:
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
};
