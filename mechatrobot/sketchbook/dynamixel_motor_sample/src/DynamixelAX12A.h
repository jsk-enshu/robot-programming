#include <SoftwareSerial.h>

// AX-12A pin1: GND ---> DC -12V AX-12A
// pin2: VDD ---> DC +12V AX-12A
// pin3: DATA ---> Tx pin of Arduino

#define OFF                         0
#define ON                          1

#define SOFT_BAUDRATE 115200

#define RX_PIN 11
#define TX_PIN 12

SoftwareSerial soft_serial(RX_PIN, TX_PIN);

unsigned char time_counter;
int error_byte;
int position_long_byte;
unsigned char position_high_byte;
unsigned char position_low_byte;
unsigned char incoming_byte;

int read_error(void)
{

    time_counter = 0;
    while((soft_serial.available() < 5) & (time_counter < 10)) // Wait for Data
        {
            time_counter++;
            delayMicroseconds(1000);
        }

    while (soft_serial.available() > 0)
        {
            incoming_byte = soft_serial.read();
            if ( (incoming_byte == 255) & (soft_serial.peek() == 255) )
                {
                    soft_serial.read();              // Start Bytes
                    soft_serial.read();              // Ax-12 ID
                    soft_serial.read();              // Length
                    error_byte = soft_serial.read(); // Error
                    return (error_byte);
                }
        }
    return (-1); // No  Response
}

int send_packet(unsigned char * packet, unsigned int length)
{
    // pinMode(TX_PIN, OUTPUT); // Switch to Transmission  Mode
    DDRB |= (1<<DDB4);

    soft_serial.write(packet, length); // Send data through sending buffer
    soft_serial.flush(); // Wait until buffer is empty (unnecessary?)

    // pinMode(TX_PIN, INPUT); // Switch back to Reception Mode
    DDRB &= ~(1<<DDB4);

    return (read_error()); // Return the read error
}

void send_packet_no_error(unsigned char * packet, unsigned int length)
{
    // pinMode(TX_PIN, OUTPUT); // Switch to Transmission  Mode
    DDRB |= (1<<DDB4);

    soft_serial.write(packet, length);  // data through sending buffer
    soft_serial.flush(); // Wait until buffer is empty (unnecessary?)

    // pinMode(TX_PIN, INPUT); // Switch back to Reception Mode
    DDRB &= ~(1<<DDB4);
}

void dynamixel_setup()
{
    // setup SoftwareSerial
    soft_serial.begin(SOFT_BAUDRATE);
}

int led_status(unsigned char id, bool status)
{
    const unsigned int length = 8;
    unsigned char packet[length] ={
        0xff,   // header1
        0xff,   // header2
        id,     // id
        4,      // length = instruction + address + status + checksum
        3,      // instruction
        25,     // P1: starting address
        status, // P2: status LED On/Off
        0       // checksum
    };

    // calc sum
    for (int i = 2; i < length-1; i++)  packet[length-1] += packet[i];
    packet[length-1] = ~packet[length-1];

    return (send_packet(packet, length));
}

int goal_position_raw(unsigned char id, int position)
{
    char position_H, position_L;
    position_H = position >> 8; // 16 bits - 2 x 8 bits variables
    position_L = position;

    const unsigned int length = 9;
    unsigned char packet[length] ={
        0xff,       // header1
        0xff,       // header2
        id,         // id
        5,          // length = instruction + address + position_L + position_H + checksum
        3,          // instruction
        30,         // P1: starting address
        position_L, // P2: target position1
        position_H, // P3: target position2
        0           // checksum
    };

    // calc sum
    for (int i = 2; i < length-1; i++)  packet[length-1] += packet[i];
    packet[length-1] = ~packet[length-1];

    return (send_packet(packet, length));
}

int goal_position(unsigned char id, float position)
{
    return goal_position_raw(id, (0x3ff & (int) (position*3.41)));
}

int read_position_raw(unsigned char id)
{
    const unsigned int length = 8;

    unsigned char packet[length] = {
        0xff, // header1
        0xff, // header2
        id,   // id
        4,    // length = instruction + address + return length + checksum
        2,    // instruction
        36,   // P1: starting address
        2,    // P2: length of data
        0     // checksum
    };
    // calc sum
    for (int i = 2; i < length-1; i++)  packet[length-1] += packet[i];
    packet[length-1] = ~packet[length-1];

    send_packet_no_error(packet, length);

    position_long_byte = -1;
    time_counter = 0;
    while((soft_serial.available() < 7) & (time_counter < 10))
        {
            time_counter++;
            delayMicroseconds(1000);
        }

    while (soft_serial.available() > 0)
        {
            incoming_byte = soft_serial.read();
            if ( (incoming_byte == 255) & (soft_serial.peek() == 255) )
                {
                    soft_serial.read();                           // Start Bytes
                    soft_serial.read();                           // Ax-12 ID
                    soft_serial.read();                           // Length
                    if( (error_byte = soft_serial.read()) != 0 )  // Error
                        return (error_byte*(-1));

                    position_low_byte = soft_serial.read();       // Position Bytes
                    position_high_byte = soft_serial.read();
                    position_long_byte = position_high_byte << 8;
                    position_long_byte = position_long_byte + position_low_byte;
                }
        }
    return (position_long_byte); // Returns the read position
}

float read_position(unsigned char id)
{
    return (read_position_raw(id)*0.293); // 300/1023=0.293
}
