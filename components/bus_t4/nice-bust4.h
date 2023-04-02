/*
  Nice BusT4
  UART communication at 19200 8n1 speed
  Before the data packet, a break is sent with a duration of 519us (10 bits)
  The contents of the packet that could be understood are described in the structure packet_cmd_body_t

  For Oview, 80 is always added to the address.
  Gate controller address unchanged.


Connection

BusT4                       ESP8266

Device Rx Tx GND
9  7  5  3  1  
10 8  6  4  2
place for cable
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- VCC (24-34V DC)


From the manual nice_dmbm_integration_protocol.pdf

• ADR: This is the address of the NICE network where the devices you want to manage are located. This can be a value from 1 to 63 (1 to 3F).
This value must be in HEX. If the destination is an integration module on DIN-BAR, this value is 0 (adr = 0) if the destination is
is an intelligent engine, this value is 1 (adr = 1).
• EPT: This is the address of the Nice engine included in the network ADR. It can be a value between 1 and 127. This value must be in HEX.
• CMD: This is the command you want to send to the destination (ADR, EPT).
• PRF: profile setup command.
• FNC: This is the function you want to send to the destination (ADR, EPT).
• EVT: This is the event that is sent to the destination (ADR, EPT).
*/

/*
  OVIEW command dumps

  SBS               55 0c 00 ff 00 66 01 05 9D 01 82 01 64 E6 0c
  STOP              55 0c 00 ff 00 66 01 05 9D 01 82 02 64 E5 0c
  OPEN              55 0c 00 ff 00 66 01 05 9D 01 82 03 00 80 0c
  CLOSE             55 0c 00 ff 00 66 01 05 9D 01 82 04 64 E3 0c
  PARENTAL OPEN 1   55 0c 00 ff 00 66 01 05 9D 01 82 05 64 E2 0c
  PARENTAL OPEN 2   55 0c 00 ff 00 66 01 05 9D 01 82 06 64 E1 0c
*/

#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h" // to add Action
#include "esphome/components/cover/cover.h"
#include <HardwareSerial.h>
#include "esphome/core/helpers.h" // parse strings with built-in tools
#include <queue>

namespace esphome {
    namespace bus_t4 {
        /* for short reference to class members */
        using namespace esphome::cover;

        static const int _UART_NO = UART0; /* uart number */
        static const int TX_P = 1;         /* pin Tx */
        static const uint32_t BAUD_BREAK = 9200; /* baudrate for a long pulse before the packet */
        static const uint32_t BAUD_WORK = 19200; /* working baudrate */
        static const uint8_t START_CODE = 0x55; /* packet start byte */


        /* esp network settings
          The series can take values from 0 to 63, by default 0
          OVIEW address starts with 8

          When networking several drives with OXI, different rows must be specified for different drives.
          In this case, the OXI must have the same row as the drive it controls.
        */

        /* Packet message type
          so far we are only interested in CMD and INF
          for the rest, I did not check the numbers
          6th byte of CMD and INF packets
        */
        enum mes_type : uint8_t {
            CMD = 0x01,  /* number verified, sending commands to automation */
            //  LSC = 0x02,  /* working with script lists */
            //  LST = 0x03,  /* work with automatic lists */
            //  POS = 0x04,  /* request and change the position of automation */
            //  GRP = 0x05,  /* sending commands to a group of automations indicating the bit mask of the motor */
            //  SCN = 0x06,  /* working with scripts */
            //  GRC = 0x07,  /* sending commands to a group of automations created through Nice Screen Configuration Tool */
            INF = 0x08,  /* returns or sets device information */
            //  LGR = 0x09,  /* working with group lists */
            //  CGR = 0x0A,  /* work with categories of groups created through Nice Screen Configuration Tool */
        };

        /*
        command menu in oview hierarchy
        9th byte of CMD packets
        */
        enum cmd_mnu : uint8_t {
            CONTROL = 0x01,
        };

        /* used in STA responses */
        enum sub_run_cmd2 : uint8_t {
            STA_OPENING = 0x02,
            STA_CLOSING = 0x03,
            OPENED = 0x04,
            CLOSED = 0x05,
            STOPPED = 0x08,
            PARTIALLY_OPENED = 0X10,
            ENDTIME = 0x06, // finished maneuver with timeout
        };

        /* Errors */
        enum errors_byte : uint8_t {
            NOERR = 0x00, // No error
            FD = 0xFD, // No command for this device
        };

        // Motor types
        enum motor_type : uint8_t {
            SLIDING = 0x01,
            SECTIONAL = 0x02,
            SWING = 0x03,
            BARRIER = 0x04,
            UPANDOVER = 0x05,
            UNKNOWN = 0x55,
        };

        // 9th byte
        enum whose_pkt : uint8_t {
            FOR_ALL = 0x00,  /* package for/from everyone */
            FOR_CU = 0x04,  /* package to/from control unit */
            FOR_OXI = 0x0A,  /* package to/from OXI receiver */
        };

        // 10th byte of GET/SET of EVT packets, only RUN was encountered for CMD packets
        enum setup_submnu : uint8_t {
            WHO = 0x04, // Who is online?
            IN1 = 0x71, // Input setup
            IN2 = 0x72, // Input setup
            IN3 = 0x73, // Input setup
            IN4 = 0x74, // Input setup
            COMM_SBS = 0x61, // Setting up commands - Step by step
            COMM_POPN = 0x62, // Command Settings - Open Partially
            COMM_OPN = 0x63, // Command settings - Open
            COMM_CLS = 0x64, // Command Settings - Close
            COMM_STP = 0x65, // Command setting - STOP
            COMM_PHOTO = 0x68, // Command setup - Photo
            COMM_PHOTO2 = 0x69, // Command settings - Photo2
            COMM_PHOTO3 = 0x6A, // Command settings - Photo3
            COMM_OPN_STP = 0x6B, // Command setting - Stop on opening
            COMM_CLS_STP = 0x6C, // Command settings - Stop on close
            COMM_LET_OPN = 0x78, // Command settings - Interference with opening
            COMM_LET_CLS = 0x79, // Command settings - Interference with closing
            OUT1 = 0x51, // Output settings
            OUT2 = 0x52, // Output settings
            LOCK_TIME = 0x5A, // Output settings - Lock operation time
            S_CUP_TIME = 0x5C, // Output Setting - Suction Cup Time
            LAMP_TIME = 0x5B, // Output settings - courtesy light time
            AUTOCLS = 0x80, // Basic Settings - Auto Close
            P_TIME = 0x81, // Main parameters - Pause time
            PH_CLS_ON = 0x84, // Main Options - Close after Photo - Active
            PH_CLS_VAR = 0x86, // Main Options - Close after Photo - Mode
            PH_CLS_TIME = 0x85, // Basic options - Close after Photo - Waiting time
            ALW_CLS_ON = 0x88, // Basic options - Always close - Active
            ALW_CLS_VAR = 0x8A, // Basic options - Always close - Mode
            ALW_CLS_TIME = 0x89, // Basic options - Always close - Timeout
            OPN_PWR = 0x4A, // Basic parameters - Force control - Opening force
            CLS_PWR = 0x4B, // Basic parameters - Force control - Closing force
            SPEED_OPN = 0x42, // Basic parameters - Speed setting - Opening speed
            SPEED_CLS = 0x43, // Basic parameters - Speed setting - Closing speed
            SPEED_SLW_OPN = 0x45, // Basic parameters - Speed setting - Slow opening speed
            SPEED_SLW_CLS = 0x46, // Basic parameters - Speed setting - Slow closing speed
            START_ON = 0x90, // Basic parameters - Start setting - Active
            START_TIME = 0x91, // Basic parameters - Start setting - Start time
            SLOW_ON = 0xA2, // Main parameters - Slowdown
            BLINK_ON = 0x94, // Basic parameters - Blink - Active
            BLINK_OPN_TIME = 0x95, // Basic parameters - Blink - Opening time
            BLINK_CLS_TIME = 0x99, // Basic parameters - Flicker - Time on closing

            TYPE_M = 0x00, // Actuator type query
            INF_STATUS = 0x01, //	Gate status (Opened/Closed/Stopped)
            MAC = 0x07, // Mac address
            MAN = 0x08, // Manufacturer
            PRD = 0x09, // Product
            HWR = 0x0a, // Hardware version
            FRM = 0x0b, // Firmware version
            DSC = 0x0c, // Description
            CUR_POS = 0x11, // Current position of automation (DPRO924 then waits for positions to be set)
            MAX_OPN = 0x12, // The maximum possible opening according to the encoder.
            POS_MAX = 0x18, // Maximum position (opening) by encoder
            POS_MIN = 0x19, // Minimum position (closing) by encoder
            INF_P_OPN1 = 0x21, //	Partial opening1
            INF_P_OPN2 = 0x22, //	Partial opening2
            INF_P_OPN3 = 0x23, //	Partial opening3
            INF_SLOW_OPN = 0x24, // Slowdown delay in opening
            INF_SLOW_CLS = 0x25, // Slowdown delay in closing
            INF_IO = 0xD1, // Input-output status

            CUR_MAN = 0x02, // Current Maneuver
            SUBMNU = 0x04, // Submenu
            STA = 0xC0, // Status in motion
            MAIN_SET = 0x80, // Main settings
            RUN = 0x82, // Command to execute
        };

        /* run cmd byte 11 of EVT packets */
        enum run_cmd : uint8_t {
            SET = 0xA9, // parameter change request
            GET = 0x99, // request to get parameters
            GET_SUPP_CMD = 0x89, // get supported commands
        };

        /* The command to be executed.
        11th byte of the CMD packet
        Used in requests and responses */
        enum control_cmd : uint8_t {
            SBS = 0x01, // Step by Step
            STOP = 0x02,   /* Stop */
            OPEN = 0x03,   /* Open */
            CLOSE = 0x04,  /* Close */
            P_OPN1 = 0x05, /* Partial opening 1 */
            P_OPN2 = 0x06, /* Partial opening 2 */
            P_OPN3 = 0x07, /* Partial opening 3 */
            RSP = 0x19, /* interface response acknowledging receipt of the command  */
            EVT = 0x29, /* interface response sending the requested information */

            P_OPN4 = 0x0b, /* Partial opening 4 - shared */
            P_OPN5 = 0x0c, /* Partial opening 5 - Priority step by step */
            P_OPN6 = 0x0d, /* Partial opening 6 - Open and block */
            UNLK_OPN = 0x19, /* Unlock and open */
            CLS_LOCK = 0x0E, /* Close and block */
            UNLCK_CLS = 0x1A, /*  Unlock and close */
            LOCK = 0x0F, /* Lock */
            UNLOCK = 0x10, /* Unlock */
            LIGHT_TIMER = 0x11, /* Light timer */
            LIGHT_SW = 0x12, /* Light on/off */
            HOST_SBS = 0x13, /* Host SBS */
            HOST_OPN = 0x14, /* Lead open */
            HOST_CLS = 0x15, /* Lead close */
            SLAVE_SBS = 0x16, /* Slave SBS */
            SLAVE_OPN = 0x17, /* Slave open */
            SLAVE_CLS = 0x18, /* Slave close */
            AUTO_ON = 0x1B, /* Auto open active */
            AUTO_OFF = 0x1C, /* Auto open inactive */
        };

/* Information for a better understanding of the composition of packets in the protocol */

// CMD request packet body
// packets with body size 0x0c=12 bytes
/*
struct packet_cmd_body_t {
  uint8_t byte_55; // Title, always 0x55
  uint8_t pct_size1; // Packet body size (without header and CRC. Total number of bytes minus three), for commands = 0x0c
  uint8_t for_series; // series to whom package ff = to all
  uint8_t for_address; // address to whom package ff = to all
  uint8_t from_series; // series from whom package
  uint8_t from_address; // address from whom the package is
  uint8_t mes_type; // message type, 1 = CMD, 8 = INF
  uint8_t mes_size; // number of bytes further minus two CRC bytes at the end, for commands = 5
  uint8_t crc1; // CRC1, XOR of the previous six bytes
  uint8_t cmd_mnu; // Command menu. cmd_mnu = 1 for control commands
  uint8_t setup_submnu; // The submenu, combined with the command group, determines the type of message to be sent.
  uint8_t control_cmd; // Command to be executed
  uint8_t offset; // Offset for responses. Affects queries like the list of supported commands
  uint8_t crc2; // crc2, XOR the previous four bytes
  uint8_t pct_size2; // packet body size (without header and CRC. Total number of bytes minus three), for commands = 0x0c

};

// RSP response packet body
// packets with body size 0x0e=14 bytes
struct packet_rsp_body_t {
  uint8_t byte_55; // Title, always 0x55
  uint8_t pct_size1; // packet body size (without header and CRC. Total number of bytes minus three), >= 0x0e
  uint8_t to_series; // series to whom package ff = to all
  uint8_t to_address; // address to whom package ff = to all
  uint8_t from_series; // series from whom package
  uint8_t from_address; // address from whom the package is
  uint8_t mes_type; // message type, for these packets always 8 = INF
  uint8_t mes_size; // number of bytes further minus two CRC bytes at the end, for commands = 5
  uint8_t crc1; // CRC1, XOR of the previous six bytes
  uint8_t cmd_mnu; // Command menu. cmd_mnu = 1 for control commands
  uint8_t sub_inf_cmd; // From which submenu the command was received. The value is 0x80 less than the original submenu
  uint8_t sub_run_cmd; // What command did you get. The value is 0x80 greater than the received command
  uint8_t hb_data; // data high bit
  uint8_t lb_data; // data low bit
  uint8_t err; // error
  uint8_t crc2; // crc2, XOR the previous four bytes
  uint8_t pct_size2; // packet body size (without header and CRC. Total number of bytes minus three), >= 0x0e

};
	
 // response packet body with EVT data
 
 struct packet_evt_body_t {
  uint8_t byte_55; // Title, always 0x55
  uint8_t pct_size1; // packet body size (without header and CRC. Total number of bytes minus three), >= 0x0e
  uint8_t to_series; // series to whom package ff = to all
  uint8_t to_address; // address to whom package ff = to all
  uint8_t from_series; // series from whom package
  uint8_t from_address; // address from whom the package is
  uint8_t mes_type; // message type, for these packets always 8 = INF
  uint8_t mes_size; // number of bytes further minus two CRC bytes at the end, for commands = 5
  uint8_t crc1; // CRC1, XOR of the previous six bytes
  uint8_t whose; // Whose package. Options: 00 - common, 04 - drive controller, 0A - OXI receiver
  uint8_t setup_submnu; // From which submenu the command was received. The value is equal to the original submenu
  uint8_t sub_run_cmd; // What command are we responding to? The value is 0x80 less than the previously sent command
  uint8_t next_data; // Next block of data
  uint8_t err; // error
  uint8_t data_blk; // Data block, can take several bytes
  uint8_t crc2; // crc2, XOR all previous bytes up to ninth (Whose packet)
  uint8_t pct_size2; // packet body size (without header and CRC. Total number of bytes minus three), >= 0x0e

};
*/

        class NiceBusT4 : public Component, public Cover {
        public:
            // drive settings
            bool autocls_flag; // Auto close - L1
            bool photocls_flag; // Close after photo - L2
            bool alwayscls_flag; // Always Close - L3

            void setup() override;

            void loop() override;

            void dump_config() override; // to display equipment information in the log

            void send_raw_cmd(std::string data);

            void send_cmd(uint8_t data) { this->tx_buffer.push(gen_control_cmd(data)); }

            void send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command, std::string next_data, bool data_on, std::string data_command); // long command
            void set_mcu(std::string command, std::string data_command); // command to the motor controller


            void set_class_gate(uint8_t class_gate) { gate_class = class_gate; }

            void set_to_address(uint16_t to_address) { this->to_addr = to_address; }

            void set_from_address(uint16_t from_address) { this->from_addr = from_address; }

            void set_oxi_address(uint16_t oxi_address) { this->oxi_addr = oxi_address; }

            /*
            // drive status acquisition interval
            void set_update_interval(uint32_t update_interval) {
              this->update_interval_ = update_interval;
            }
            */
            void set_update_interval(uint32_t update_interval) {
                this->status_update_interval = update_interval;
            }

            cover::CoverTraits get_traits() override;

        protected:
            void control(const cover::CoverCall &call) override;

            uint32_t last_detect_millis = 0;
            uint32_t last_received_byte_millis = 0;
            uint32_t last_received_status_millis = 0;
            uint32_t status_update_interval = 0;

            uint8_t gate_class = UNKNOWN;

            CoverOperation last_published_current_operation = COVER_OPERATION_IDLE;
            float last_published_position;

            // variables for uart
            uart_t *_uart = nullptr;

            uint16_t _max_opn = 0;  // maximum encoder or timer position
            uint16_t _pos_opn = 2048;  // encoder or timer opening position, not for all drives
            uint16_t _pos_cls = 0;  // encoder or timer close position, not for all drives
            uint16_t _pos_usl = 0;  // conditional current position of encoder or timer, not for all drives
            // packet header settings
            uint16_t from_addr = 0x0066; // from whom the packet is, bust4 gateway address
            uint16_t to_addr; // = 0x00ff; // to whom the packet is, the address of the drive controller that we control
            uint16_t oxi_addr; // = 0x000a; // receiver address

            std::vector<uint8_t> raw_cmd_prepare(std::string data); // preparation of user-entered data for the possibility of sending

            // inf command generation
            std::vector<uint8_t> gen_inf_cmd(uint8_t to_addr1, uint8_t to_addr2, uint8_t whose, uint8_t inf_cmd, uint8_t run_cmd, uint8_t next_data, const std::vector<uint8_t> &data, size_t len);
            // for commands without data
            std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd) {
                return gen_inf_cmd((uint8_t) (this->to_addr >> 8), (uint8_t) (this->to_addr & 0xFF), whose, inf_cmd, run_cmd, 0x00, {0x00}, 0);
            }
            // for commands with data
            std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, std::vector<uint8_t> data) {
                return gen_inf_cmd((uint8_t) (this->to_addr >> 8), (uint8_t) (this->to_addr & 0xFF), whose, inf_cmd, run_cmd, next_data, data, data.size());
            }
            // for commands with address and without data
            std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data) {
                return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, next_data, {0x00}, 0);
            }

            // generating cmd commands
            std::vector<uint8_t> gen_control_cmd(uint8_t control_cmd);

            void init_device(uint8_t addr1, uint8_t addr2, uint8_t device);

            void send_array_cmd(std::vector<uint8_t> data);
            void send_array_cmd(const uint8_t *data, size_t len);

            void parse_status_packet(const std::vector<uint8_t> &data); // parsing the status package

            void handle_received_byte(uint8_t c); // received byte handler
            bool validate_received_message(); // function of checking the received message

            bool supports_querying_position() const;

            void query_status();

            void publish_state_if_changed();

            std::vector<uint8_t> rx_buffer;
            std::queue<std::vector<uint8_t>> tx_buffer;
            bool ready_to_tx = true;

            std::vector<uint8_t> manufacturer_;
            std::vector<uint8_t> product_;
            std::vector<uint8_t> hardware_;
            std::vector<uint8_t> firmware_;
            std::vector<uint8_t> description_;
            std::vector<uint8_t> oxi_product;
            std::vector<uint8_t> oxi_hardware;
            std::vector<uint8_t> oxi_firmware;
            std::vector<uint8_t> oxi_description;
        };
    }
}
