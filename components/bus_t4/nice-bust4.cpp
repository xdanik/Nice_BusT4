#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // to use string helper functions

namespace esphome {
    namespace bus_t4 {

        static const char *TAG = "bus_t4.cover";

        using namespace esphome::cover;

        CoverTraits NiceBusT4::get_traits() {
            auto traits = CoverTraits();
            traits.set_supports_position(true);
            return traits;
        }

        void NiceBusT4::control(const CoverCall &call) {
            if (call.get_stop()) {
                // uint8_t data[2] = {CONTROL, STOP};
                this->tx_buffer.push(gen_control_cmd(STOP));
                this->tx_buffer.push(gen_inf_cmd(FOR_CU, INF_STATUS, GET)); // Gate status (Open/Closed/Stopped)
                this->tx_buffer.push(gen_inf_cmd(FOR_CU, CUR_POS, GET)); // query of the conditional current position of the actuator
            } else if (call.get_position().has_value()) {
                auto pos = *call.get_position();
                if (pos != this->position) {
                    if (pos == COVER_OPEN) {
                        if (this->current_operation != COVER_OPERATION_OPENING) {
                            this->tx_buffer.push(gen_control_cmd(OPEN));
                        }
                    } else if (pos == COVER_CLOSED) {
                        if (this->current_operation != COVER_OPERATION_CLOSING) {
                            this->tx_buffer.push(gen_control_cmd(CLOSE));
                        }
                    }/* else {
                      uint8_t data[3] = {CONTROL, SET_POSITION, (uint8_t)(pos * 100)};
                      this->send_command_(data, 3);
                    }*/
                }
            }
        }

        void NiceBusT4::setup() {
            _uart = uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
            delay(500);

            // who is online?
            this->tx_buffer.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        }

        void NiceBusT4::loop() {
            // retry detection every 10 seconds
            if (this->gate_class == UNKNOWN) {
                if ((millis() - this->last_detect_millis) > 10000) {
                    // who is online?
                    this->tx_buffer.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
                    this->last_detect_millis = millis();
                }
            }

            while (uart_rx_available(_uart) > 0) {
                uint8_t c = (uint8_t) uart_read_char(_uart);
                this->last_received_byte_millis = millis();
                this->handle_received_byte(c);
            }

            // allow sending only after 100ms since last received message
            if (millis() - this->last_received_byte_millis > 100) {
                this->ready_to_tx = true;
            }

            if (this->ready_to_tx && !this->tx_buffer.empty()) {
                // send the first command in the queue
                this->send_array_cmd(this->tx_buffer.front());
                this->tx_buffer.pop();
                this->ready_to_tx = false;
            }
        }

        void NiceBusT4::handle_received_byte(uint8_t c) {
            this->rx_buffer.push_back(c);
            if (!this->validate_received_message()) {
                this->rx_buffer.clear();
            }
        }

        bool NiceBusT4::validate_received_message() {
            uint32_t at = this->rx_buffer.size() - 1; // position of the last received byte
            uint8_t *data = &this->rx_buffer[0]; // pointer to the first byte of the message

            // Byte 0: HEADER1 (Always 0x00)
            if (at == 0) {
                return data[0] == 0x00;
            }
            // Byte 1: HEADER2 (Always 0x55)
            if (at == 1) {
                return data[1] == START_CODE;
            }

            // Byte 2: packet_size - number of bytes further + 1
            // Check is not carried out
            if (at == 2) {
                return true;
            }
            uint8_t packet_size = data[2];
            uint8_t length = (packet_size + 3);

            // Byte 3: Series (row) to whom package
            // Byte 4: Address to whom the package is for
            // Byte 5: Series (row) from whom the package
            // Byte 6: Address from whom the package is from
            // Byte 7: Message type CMD or INF
            // Byte 8: The number of bytes to follow minus the two CRC bytes at the end.
            if (at <= 8) {
                return true;
            }

            uint8_t crc1 = (data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8]);

            // Byte 9: crc1 = XOR (Byte 3 : Byte 8) XOR six previous bytes
            if (at == 9 && data[9] != crc1) {
                ESP_LOGW(TAG, "Received invalid message checksum 1 %02X!=%02X", data[9], crc1);
                return false;
            }

            // wait until all data of the package arrives
            if (at < length) {
                return true;
            }

            // count crc2
            uint8_t crc2 = data[10];
            for (uint8_t i = 11; i < length - 1; i++) {
                crc2 = (crc2 ^ data[i]);
            }
            if (data[length - 1] != crc2) {
                ESP_LOGW(TAG, "Received invalid message checksum 2 %02X!=%02X", data[length - 1], crc2);
                return false;
            }

            // Byte Last: packet_size
            if (data[length] != packet_size) {
                ESP_LOGW(TAG, "Received invalid message size %02X!=%02X", data[length], packet_size);
                return false;
            }

            // the correct message was received and lies in the rx_buffer buffer

            // Remove 0x00 from the beginning of the message
            rx_buffer.erase(rx_buffer.begin());

            std::string pretty_cmd = format_hex_pretty(rx_buffer);
            ESP_LOGI(TAG, "Package received: %S ", pretty_cmd.c_str());

            parse_status_packet(rx_buffer);

            // return false to reset rx buffer
            return false;
        }

        void NiceBusT4::parse_status_packet(const std::vector<uint8_t> &data) {
            if ((data[1] == 0x0d) && (data[13] == 0xFD)) { // error
                ESP_LOGE(TAG, "Command not available for this device");
            }

            if ((data[1] == (data[12] + 0xd)) && (data[13] == NOERR)) { // if evt
                ESP_LOGD(TAG, "An EVT data packet has been received. Data size %d ", data[12]);
                std::vector<uint8_t> vec_data(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                std::string str(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                ESP_LOGI(TAG, "Data line: %S ", str.c_str());
                std::string pretty_data = format_hex_pretty(vec_data);
                ESP_LOGI(TAG, "HEX data %S ", pretty_data.c_str());
                // received a package with EVT data, we begin to disassemble

                // interested in responses to GET requests that came without errors from the drive
                if ((data[6] == INF) && (data[9] == FOR_CU) && (data[11] == GET - 0x80) && (data[13] == NOERR)) {
                    ESP_LOGI(TAG, "Request response received %X ", data[10]);
                    switch (data[10]) {
                        case TYPE_M:
                            //           ESP_LOGI(TAG,  "type of drive %X",  data[14]);
                            // switch 14
                            switch (data[14]) {
                                case SLIDING:
                                    this->gate_class = SLIDING;
                                    //        ESP_LOGD(TAG, "Gate type: Sliding %#X ", data[14]);
                                    break;
                                case SECTIONAL:
                                    this->gate_class = SECTIONAL;
                                    //        ESP_LOGD(TAG, "Gate type: Sectional %#X ", data[14]);
                                    break;
                                case SWING:
                                    this->gate_class = SWING;
                                    //        ESP_LOGD(TAG, "Gate type: Swing %#X ", data[14]);
                                    break;
                                case BARRIER:
                                    this->gate_class = BARRIER;
                                    //        ESP_LOGD(TAG, "Gate type: Barrier %#X ", data[14]);
                                    break;
                                case UPANDOVER:
                                    this->gate_class = UPANDOVER;
                                    //        ESP_LOGD(TAG, "Gate type: up-and-over %#X ", data[14]);
                                    break;
                            }
                            break;
                        case INF_IO: // response to a request for the position of the sliding gate limit switch
                            switch (data[16]) { //16
                                case 0x00:
                                    ESP_LOGI(TAG, "  End switch didn't work ");
                                    break; // 0x00
                                case 0x01:
                                    ESP_LOGI(TAG, "  End switch for closing ");
                                    this->position = COVER_CLOSED;
                                    break; //  0x01
                                case 0x02:
                                    ESP_LOGI(TAG, "  End switch for opening ");
                                    this->position = COVER_OPEN;
                                    break; // 0x02

                            }  // switch 16
                            this->publish_state();

                            break; //  INF_IO

                            // encoder maximum opening position, open, close
                        case MAX_OPN:
                            this->_max_opn = (data[14] << 8) + data[15];
                            ESP_LOGI(TAG, "Maximum encoder position: %d", this->_max_opn);
                            break;

                        case POS_MIN:
                            this->_pos_cls = (data[14] << 8) + data[15];
                            ESP_LOGI(TAG, "Closed gate position: %d", this->_pos_cls);
                            break;

                        case POS_MAX:
                            if (((data[14] << 8) + data[15]) > 0x00) { // if the response from the actuator contains data on the opening position
                                this->_pos_opn = (data[14] << 8) + data[15];
                            }
                            ESP_LOGI(TAG, "Opened gate position: %d", this->_pos_opn);
                            break;

                        case CUR_POS:
                            this->_pos_usl = (data[14] << 8) + data[15];
                            this->position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
                            ESP_LOGI(TAG, "Current gate position: %d, position in %%: %f", _pos_usl, (_pos_usl - _pos_cls) * 100.0f / (_pos_opn - _pos_cls));
                            this->publish_state();
                            break;

                        case 0x01:
                            switch (data[14]) {
                                case OPENED:
                                    ESP_LOGI(TAG, "  gate open");
                                    this->position = COVER_OPEN;
                                    this->current_operation = COVER_OPERATION_IDLE;
                                    break;
                                case CLOSED:
                                    ESP_LOGI(TAG, "  gate close");
                                    this->position = COVER_CLOSED;
                                    this->current_operation = COVER_OPERATION_IDLE;
                                    break;
                                case 0x01:
                                    ESP_LOGI(TAG, "  gate stopped");
                                    this->current_operation = COVER_OPERATION_IDLE;
                                    //          this->position = COVER_OPEN;
                                    break;
                            }  // switch
                            this->publish_state();
                            break;

                        case AUTOCLS:
                            this->autocls_flag = data[14];
                            break;

                        case PH_CLS_ON:
                            this->photocls_flag = data[14];
                            break;

                        case ALW_CLS_ON:
                            this->alwayscls_flag = data[14];
                            break;
                    }
                }

                // if responses to GET requests that came without errors from the drive
                if ((data[6] == INF) && (data[9] == FOR_CU) && (data[11] == SET - 0x80) && (data[13] == NOERR)) { // interested in responses to SET requests that came without errors from the drive
                    switch (data[10]) { // cmd_submnu
                        case AUTOCLS:
                            tx_buffer.push(gen_inf_cmd(FOR_CU, AUTOCLS, GET)); // Auto close
                            break;

                        case PH_CLS_ON:
                            tx_buffer.push(gen_inf_cmd(FOR_CU, PH_CLS_ON, GET)); // Close after photo
                            break;

                        case ALW_CLS_ON:
                            tx_buffer.push(gen_inf_cmd(FOR_CU, ALW_CLS_ON, GET)); // always close
                            break;
                    }
                }

                // if responses to SET requests that came without errors from the drive
                if ((data[6] == INF) && (data[9] == FOR_ALL) && (data[11] == GET - 0x80) && (data[13] == NOERR)) { // interested in FOR_ALL responses to GET requests that came without errors

                    switch (data[10]) {
                        case MAN:
                            //       ESP_LOGCONFIG(TAG, "  Производитель: %S ", str.c_str());
                            this->manufacturer_.assign(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                            break;
                        case PRD:
                            if (((uint8_t) (this->oxi_addr >> 8) == data[4]) && ((uint8_t) (this->oxi_addr & 0xFF) == data[5])) { // if the packet is from the receiver
                                // ESP_LOGCONFIG(TAG, "  Receiver: %S ", str.c_str());
                                this->oxi_product.assign(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                            } else if (((uint8_t) (this->to_addr >> 8) == data[4]) && ((uint8_t) (this->to_addr & 0xFF) == data[5])) { // if the package is from the drive controller
                                // ESP_LOGCONFIG(TAG, "  Drive unit: %S ", str.c_str());
                                this->product_.assign(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                            }
                            break;
                        case HWR:
                            if (((uint8_t) (this->oxi_addr >> 8) == data[4]) && ((uint8_t) (this->oxi_addr & 0xFF) == data[5])) { // if the packet is from the receiver
                                this->oxi_hardware.assign(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                            } else if (((uint8_t) (this->to_addr >> 8) == data[4]) && ((uint8_t) (this->to_addr & 0xFF) == data[5])) { // if the package is from the drive controller
                                this->hardware_.assign(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                            }
                            break;
                        case FRM:
                            if (((uint8_t) (this->oxi_addr >> 8) == data[4]) && ((uint8_t) (this->oxi_addr & 0xFF) == data[5])) { // if the packet is from the receiver
                                this->oxi_firmware.assign(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                            } else if (((uint8_t) (this->to_addr >> 8) == data[4]) && ((uint8_t) (this->to_addr & 0xFF) == data[5])) { // if the package is from the drive controller
                                this->firmware_.assign(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                            }
                            break;
                        case DSC:
                            if (((uint8_t) (this->oxi_addr >> 8) == data[4]) && ((uint8_t) (this->oxi_addr & 0xFF) == data[5])) { // if the packet is from the receiver
                                this->oxi_description.assign(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                            } else if (((uint8_t) (this->to_addr >> 8) == data[4]) && ((uint8_t) (this->to_addr & 0xFF) == data[5])) { // if the package is from the drive controller
                                this->description_.assign(this->rx_buffer.begin() + 14, this->rx_buffer.end() - 2);
                            }
                            break;
                        case WHO:
                            if (data[12] == 0x01) {
                                if (data[14] == 0x04) { // drive unit
                                    this->to_addr = ((uint16_t) data[4] << 8) | data[5];
                                    init_device(data[4], data[5], data[14]);
                                } else if (data[14] == 0x0A) { // receiver
                                    this->oxi_addr = ((uint16_t) data[4] << 8) | data[5];
                                    init_device(data[4], data[5], data[14]);
                                }
                            }
                            break;
                    }
                }

                // packets from the receiver with information about the list of remotes that came without errors
                if ((data[9] == 0x0A) && (data[10] == 0x25) && (data[11] == 0x01) && (data[12] == 0x0A) && (data[13] == NOERR)) {
                    ESP_LOGCONFIG(TAG, "Remote Number: %X%X%X%X, Command: %X, Button: %X, Mode: %X, Hit Count: %d", vec_data[5], vec_data[4], vec_data[3], vec_data[2], vec_data[8] / 0x10, vec_data[5] / 0x10, vec_data[7] + 0x01,
                                  vec_data[6]);
                }

                // packets from the receiver with information about the remote button read
                if ((data[9] == 0x0A) && (data[10] == 0x26) && (data[11] == 0x41) && (data[12] == 0x08) && (data[13] == NOERR)) {
                    ESP_LOGCONFIG(TAG, "Button %X, remote control number: %X%X%X%X", vec_data[0] / 0x10, vec_data[0] % 0x10, vec_data[1], vec_data[2], vec_data[3]);
                }
            } else if ((data[14] == NOERR) && (data[1] > 0x0d)) { // otherwise, the Responce packet - confirmation of the received command
                ESP_LOGD(TAG, "RSP package received");
                std::vector<uint8_t> vec_data(this->rx_buffer.begin() + 12, this->rx_buffer.end() - 3);
                std::string str(this->rx_buffer.begin() + 12, this->rx_buffer.end() - 3);
                ESP_LOGI(TAG, "Data line: %S ", str.c_str());
                std::string pretty_data = format_hex_pretty(vec_data);
                ESP_LOGI(TAG, "HEX data %S ", pretty_data.c_str());
                switch (data[9]) { // cmd_mnu
                    case FOR_CU:
                        ESP_LOGI(TAG, "Drive controller package");
                        // sub_inf_cmd
                        switch (data[10] + 0x80) {
                            case RUN:
                                ESP_LOGI(TAG, "RUN submenu");

                                // sub_run_cmd1
                                switch (data[11] - 0x80) {
                                    case SBS:
                                        ESP_LOGI(TAG, "Command: step by step");
                                        break;
                                    case STOP:
                                        ESP_LOGI(TAG, "Command: STOP");
                                        break;
                                    case OPEN:
                                        ESP_LOGI(TAG, "Command: OPEN");
                                        this->current_operation = COVER_OPERATION_OPENING;
                                        break;
                                    case CLOSE:
                                        ESP_LOGI(TAG, "Command: CLOSE");
                                        this->current_operation = COVER_OPERATION_CLOSING;
                                        break;
                                    case P_OPN1:
                                        ESP_LOGI(TAG, "Command: Partial opening");
                                        break;
                                    case STOPPED:
                                        this->current_operation = COVER_OPERATION_IDLE;
                                        ESP_LOGI(TAG, "Command: Stopped");
                                        break;
                                    case ENDTIME:
                                        ESP_LOGI(TAG, "Operation completed with timeout");
                                        break;
                                }

                                // sub_run_cmd2
                                switch (data[11]) {
                                    case STA_OPENING:
                                        ESP_LOGI(TAG, "Operation: Opening");
                                        this->current_operation = COVER_OPERATION_OPENING;
                                        break;
                                    case STA_CLOSING:
                                        ESP_LOGI(TAG, "Operation: Closing");
                                        this->current_operation = COVER_OPERATION_CLOSING;
                                        break;
                                    case CLOSED:
                                        ESP_LOGI(TAG, "Operation: Closed");
                                        this->position = COVER_CLOSED;
                                        this->current_operation = COVER_OPERATION_IDLE;
                                        break;
                                    case OPENED:
                                        this->position = COVER_OPEN;
                                        ESP_LOGI(TAG, "Operation: Opened");
                                        this->current_operation = COVER_OPERATION_IDLE;
                                        // calibrate opened possition if the motor does not report max supported position
                                        if (this->_max_opn == 0) {
                                            this->_pos_opn = this->_pos_usl;
                                            ESP_LOGI(TAG, "Opened position calibrated");
                                        }
                                        break;
                                    case STOPPED:
                                    case 0x10: // partialy oppened
                                        this->current_operation = COVER_OPERATION_IDLE;
                                        ESP_LOGI(TAG, "Operation: Stopped");
                                        break;
                                    default:
                                        ESP_LOGI(TAG, "Operation: %X", data[11]);
                                }
                                this->publish_state();
                                break;

                            case STA:
                                ESP_LOGI(TAG, "Submenu status in motion");
                                switch (data[11]) {
                                    case STA_OPENING:
                                    case 0x83:
                                        ESP_LOGI(TAG, "Movement: Opening");
                                        this->current_operation = COVER_OPERATION_OPENING;
                                        break;
                                    case STA_CLOSING:
                                    case 0x84:
                                        ESP_LOGI(TAG, "Movement: Closing");
                                        this->current_operation = COVER_OPERATION_CLOSING;
                                        break;
                                    case CLOSED:
                                        ESP_LOGI(TAG, "Movement: Closed");
                                        this->position = COVER_CLOSED;
                                        this->current_operation = COVER_OPERATION_IDLE;
                                        break;
                                    case OPENED:
                                        this->position = COVER_OPEN;
                                        ESP_LOGI(TAG, "Movement: Opened");
                                        this->current_operation = COVER_OPERATION_IDLE;
                                        break;
                                    case STOPPED:
                                        this->current_operation = COVER_OPERATION_IDLE;
                                        ESP_LOGI(TAG, "Movement: Stopped");
                                        break;
                                    default:
                                        ESP_LOGI(TAG, "Movement: %X", data[11]);
                                }

                                this->_pos_usl = (data[12] << 8) + data[13];
                                this->position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
                                ESP_LOGD(TAG, "Current gate position: %d, position in %%: %f", _pos_usl, (_pos_usl - _pos_cls) * 100.0f / (_pos_opn - _pos_cls));
                                this->publish_state();
                                break;
                            default:
                                ESP_LOGI(TAG, "Submenu %X", data[10]);
                        }

                        break;
                    case CONTROL:
                        ESP_LOGI(TAG, "CONTROL package");
                        break;
                    case FOR_ALL:
                        ESP_LOGI(TAG, "Package for everyone");
                        break;
                    case 0x0A:
                        ESP_LOGI(TAG, "Receiver package");
                        break;
                    default:
                        ESP_LOGI(TAG, "Menu %X", data[9]);
                }
            }
        }

        // print detected config
        void NiceBusT4::dump_config() {
            ESP_LOGCONFIG(TAG, "  Bus T4 Cover");
            /*ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", *this->header_[1], *this->header_[2]);*/
            switch (this->gate_class) {
                case SLIDING:
                    ESP_LOGCONFIG(TAG, "  Type: Sliding gate");
                    break;
                case SECTIONAL:
                    ESP_LOGCONFIG(TAG, "  Type: Sectional door");
                    break;
                case SWING:
                    ESP_LOGCONFIG(TAG, "  Type: Swing gate");
                    break;
                case BARRIER:
                    ESP_LOGCONFIG(TAG, "  Type: Barrier");
                    break;
                case UPANDOVER:
                    ESP_LOGCONFIG(TAG, "  Type: Up-and-over gate");
                    break;
                default:
                    ESP_LOGCONFIG(TAG, "  Type: Unknown gate, 0x%02X", this->gate_class);
            }

            ESP_LOGCONFIG(TAG, "  Maximum encoder or timer position: %d", this->_max_opn);
            ESP_LOGCONFIG(TAG, "  Opened gate position: %d", this->_pos_opn);
            ESP_LOGCONFIG(TAG, "  Closed gate position: %d", this->_pos_cls);

            std::string manuf_str(this->manufacturer_.begin(), this->manufacturer_.end());
            ESP_LOGCONFIG(TAG, "  Manufacturer: %S ", manuf_str.c_str());

            std::string prod_str(this->product_.begin(), this->product_.end());
            ESP_LOGCONFIG(TAG, "  Drive unit: %S ", prod_str.c_str());

            std::string hard_str(this->hardware_.begin(), this->hardware_.end());
            ESP_LOGCONFIG(TAG, "  Drive hardware: %S ", hard_str.c_str());

            std::string firm_str(this->firmware_.begin(), this->firmware_.end());
            ESP_LOGCONFIG(TAG, "  Drive firmware: %S ", firm_str.c_str());

            std::string dsc_str(this->description_.begin(), this->description_.end());
            ESP_LOGCONFIG(TAG, "  Drive description: %S ", dsc_str.c_str());


            ESP_LOGCONFIG(TAG, "  Gateway address: 0x%04X", from_addr);
            ESP_LOGCONFIG(TAG, "  Drive address: 0x%04X", to_addr);
            ESP_LOGCONFIG(TAG, "  Receiver address: 0x%04X", oxi_addr);

            std::string oxi_prod_str(this->oxi_product.begin(), this->oxi_product.end());
            ESP_LOGCONFIG(TAG, "  Receiver: %S ", oxi_prod_str.c_str());

            std::string oxi_hard_str(this->oxi_hardware.begin(), this->oxi_hardware.end());
            ESP_LOGCONFIG(TAG, "  Receiver hardware: %S ", oxi_hard_str.c_str());

            std::string oxi_firm_str(this->oxi_firmware.begin(), this->oxi_firmware.end());
            ESP_LOGCONFIG(TAG, "  Receiver firmware: %S ", oxi_firm_str.c_str());

            std::string oxi_dsc_str(this->oxi_description.begin(), this->oxi_description.end());
            ESP_LOGCONFIG(TAG, "  Receiver description: %S ", oxi_dsc_str.c_str());

            ESP_LOGCONFIG(TAG, "  Auto close - L1: %S ", autocls_flag ? "Yes" : "No");
            ESP_LOGCONFIG(TAG, "  Close after photo - L2: %S ", photocls_flag ? "Yes" : "No");
            ESP_LOGCONFIG(TAG, "  Always close - L3: %S ", alwayscls_flag ? "Yes" : "No");
        }

        // build control command
        std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
            std::vector<uint8_t> frame = {(uint8_t) (this->to_addr >> 8), (uint8_t) (this->to_addr & 0xFF), (uint8_t) (this->from_addr >> 8), (uint8_t) (this->from_addr & 0xFF)}; // header
            frame.push_back(CMD);  // 0x01
            frame.push_back(0x05);
            uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
            frame.push_back(crc1);
            frame.push_back(CONTROL);
            frame.push_back(RUN);
            frame.push_back(control_cmd);
            frame.push_back(0x64); // OFFSET CMD (most drives works with 0x00 but DPRO924 requires 0x64)
            uint8_t crc2 = (frame[7] ^ frame[8] ^ frame[9] ^ frame[10]);
            frame.push_back(crc2);
            uint8_t f_size = frame.size();
            frame.push_back(f_size);
            frame.insert(frame.begin(), f_size);
            frame.insert(frame.begin(), START_CODE);

            return frame;
        }

        // generating an INF command with and without data
        std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len) {
            std::vector<uint8_t> frame = {to_addr1, to_addr2, (uint8_t) (this->from_addr >> 8), (uint8_t) (this->from_addr & 0xFF)}; // заголовок
            frame.push_back(INF);  // 0x08 mes_type
            frame.push_back(0x06 + len); // mes_size
            uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
            frame.push_back(crc1);
            frame.push_back(whose);
            frame.push_back(inf_cmd);
            frame.push_back(run_cmd);
            frame.push_back(next_data);
            frame.push_back(len);
            if (len > 0) {
                frame.insert(frame.end(), data.begin(), data.end()); // data block
            }
            uint8_t crc2 = frame[7];
            for (size_t i = 8; i < 12 + len; i++) {
                crc2 = crc2 ^ frame[i];
            }
            frame.push_back(crc2);
            uint8_t f_size = frame.size();
            frame.push_back(f_size);
            frame.insert(frame.begin(), f_size);
            frame.insert(frame.begin(), START_CODE);

            return frame;

        }

        void NiceBusT4::send_raw_cmd(std::string data) {
            std::vector<uint8_t> v_cmd = raw_cmd_prepare(data);
            send_array_cmd(&v_cmd[0], v_cmd.size());
        }

        std::vector<uint8_t> NiceBusT4::raw_cmd_prepare(std::string data) {
            // remove everything except letters and numbers
            data.erase(remove_if(data.begin(), data.end(), [](const unsigned char ch) {
                return (!(iswalnum(ch)));
            }), data.end());

            std::vector<uint8_t> frame;
            frame.resize(0); // reset command size

            // fill command array
            for (uint8_t i = 0; i < data.size(); i += 2) {
                std::string sub_str(data, i, 2); // take 2 bytes from the command
                char hexstoi = (char) std::strtol(&sub_str[0], 0, 16); // convert to number
                frame.push_back(hexstoi);  // write the number to the element of the new command line
            }

            return frame;
        }

        void NiceBusT4::send_array_cmd(std::vector<uint8_t> data) {
            return send_array_cmd((const uint8_t *) data.data(), data.size());
        }

        void NiceBusT4::send_array_cmd(const uint8_t *data, size_t len) {
            char br_ch = 0x00;
            uart_flush(_uart);

            // send break at lower baund rate
            uart_set_baudrate(_uart, BAUD_BREAK);
            uart_write(_uart, &br_ch, 1); // send zero at low speed
            uart_wait_tx_empty(_uart); // wait until the sending is completed. There is an error in the uart.h library (esp8266 core 3.0.2), waiting is not enough for further uart_set_baudrate().
            delayMicroseconds(90); // add a delay to the wait, otherwise the speed will switch before sending. With a delay on d1-mini, I got the perfect signal, break = 520us

            // send payload itself
            uart_set_baudrate(_uart, BAUD_WORK);
            uart_write(_uart, (char *) &data[0], len);
            uart_wait_tx_empty(_uart); // waiting for the completion of sending

            // print to log
            std::string pretty_cmd = format_hex_pretty((uint8_t *) &data[0], len);
            ESP_LOGI(TAG, "Sent: %S ", pretty_cmd.c_str());
        }

        // generating and sending inf commands from yaml configuration
        void NiceBusT4::send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command, std::string next_data, bool data_on, std::string data_command) {
            std::vector<uint8_t> v_to_addr = raw_cmd_prepare(to_addr);
            std::vector<uint8_t> v_whose = raw_cmd_prepare(whose);
            std::vector<uint8_t> v_command = NiceBusT4::raw_cmd_prepare(command);
            std::vector<uint8_t> v_type_command = raw_cmd_prepare(type_command);
            std::vector<uint8_t> v_next_data = raw_cmd_prepare(next_data);
            std::vector<uint8_t> v_data_command = raw_cmd_prepare(data_command);

            if (data_on) {
                tx_buffer.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0], v_data_command, v_data_command.size()));
            } else {
                tx_buffer.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0]));
            }
        }

        void NiceBusT4::set_mcu(std::string command, std::string data_command) {
            std::vector<uint8_t> v_command = raw_cmd_prepare(command);
            std::vector<uint8_t> v_data_command = raw_cmd_prepare(data_command);
            tx_buffer.push(gen_inf_cmd(0x04, v_command[0], 0xa9, 0x00, v_data_command));
        }

        // device initialization
        void NiceBusT4::init_device(const uint8_t addr1, const uint8_t addr2, const uint8_t device) {
            if (device == FOR_CU) {
                tx_buffer.push(gen_inf_cmd(addr1, addr2, device, TYPE_M, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, FOR_ALL, MAN, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, device, POS_MAX, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, device, POS_MIN, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, device, CUR_POS, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, device, INF_STATUS, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, device, AUTOCLS, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, device, PH_CLS_ON, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, device, ALW_CLS_ON, GET, 0x00));
            }
            if (device == FOR_OXI) {
                tx_buffer.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00));
                tx_buffer.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00));
            }
        }
    }
}
