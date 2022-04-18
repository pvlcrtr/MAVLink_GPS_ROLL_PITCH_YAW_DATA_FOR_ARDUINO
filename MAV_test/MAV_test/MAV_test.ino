/*
 Name:		MAV_test.ino
 Created:	4/18/2022 9:30:07 PM
 Author:	Pavel Duniak
 //https://mavlink.io/en/messages/common.html#messages

*/



// In case we need a second serial port for debugging
#define SOFT_SERIAL_DEBUGGING   // Comment this line if no serial debugging is needed
#ifdef SOFT_SERIAL_DEBUGGING

// Library to use serial debugging with a second board
#include <SoftwareSerial.h>

#define RxPin0 9
#define TxPin0 10

int request = 0;
int receive = 0;

SoftwareSerial pxSerial = SoftwareSerial(RxPin0, TxPin0);  // RX, TX || UNO - PX4

#endif

#include "mavlink.h"
//#include "common/mavlink_msg_request_data_stream.h"


// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

void setup() {

    pinMode(RxPin0, INPUT);
    pinMode(TxPin0, OUTPUT);

    // MAVLink interface start
    // Serial.begin(57600);

#ifdef SOFT_SERIAL_DEBUGGING
  // [DEB] Soft serial port start
    Serial.begin(57600);
    pxSerial.begin(57600);
    Serial.println("MAVLink starting.");
#endif
}

void loop() {

    // MAVLink
    /* The default UART header for your MCU */
    int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
    int compid = 158;                ///< The component sending the message
    int type = MAV_TYPE_FIXED_WING;   ///< This system is an airplane / fixed wing

    // Define the system type, in this case an airplane -> on-board controller
    // uint8_t system_type = MAV_TYPE_FIXED_WING;
    uint8_t system_type = MAV_TYPE_GENERIC;
    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

    uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
    uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message with the standard UART send function
    // uart0_send might be named differently depending on
    // the individual microcontroller / library in use.
    unsigned long currentMillisMAVLink = millis();
    if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
        previousMillisMAVLink = currentMillisMAVLink;

#ifdef SOFT_SERIAL_DEBUGGING
        pxSerial.write(buf, len);
        //Serial.println("Ardu HB");
#else
        Serial.write(buf, len);
#endif

        //Mav_Request_Data();
        num_hbs_pasados++;
        if (num_hbs_pasados >= num_hbs) {
            // Request streams from Pixhawk
#ifdef SOFT_SERIAL_DEBUGGING
            Serial.println("Streams requested!");
#endif
            Mav_Request_Data();
            num_hbs_pasados = 0;
        }

    }

    // Check reception buffer
    comm_receive();

    //  Serial.print("receive:  ");
    //  Serial.print(receive);
    //  Serial.print("       request:  ");
    //  Serial.println(request);
}

void Mav_Request_Data()
{

    request = request + 1;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];


    // STREAMS that can be requested
    /*
       Definitions are in common.h: enum MAV_DATA_STREAM

       MAV_DATA_STREAM_ALL=0, // Enable all data streams
       MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
       MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
       MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
       MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
       MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
       MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
       MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
       MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
       MAV_DATA_STREAM_ENUM_END=13,

       Data in PixHawk available in:
        - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
        - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
    */

    // To be setup according to the needed information to be requested from the Pixhawk
    const int  maxStreams = 4;
    const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_RAW_CONTROLLER };
    const uint16_t MAVRates[maxStreams] = { 0x01, 0x01, 0x01, 0x01};


    for (int i = 0; i < maxStreams; i++)

    {
        mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
#ifdef SOFT_SERIAL_DEBUGGING
        pxSerial.write(buf, len);
#else
        Serial.write(buf, len);
#endif
    }

}



void comm_receive()

{

    receive = receive + 1;

    mavlink_message_t msg;
    mavlink_status_t status;

    // Echo for manual debugging
    // Serial.println("---Start---");

#ifdef SOFT_SERIAL_DEBUGGING
    while (pxSerial.available() > 0) {
        uint8_t c = pxSerial.read();
#else
    while (Serial.available() > 0) {
        uint8_t c = Serial.read();
#endif

        // Try to get a new message
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

            // Handle message
            switch (msg.msgid) 
            
            {

            case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
            {
                // E.g. read GCS heartbeat and go into
                // comm lost mode if timer times out
#ifdef SOFT_SERIAL_DEBUGGING
                //Serial.println("PX HB");
#endif
            }
            break;


            //--------------------------------------------Gauname Vz,Vy,Vx bet nezinau ar butent tokie turi buti-------------------------------------------------------------

/*
            //GLOBAL_POSITION_INT:  // #33: PARAM_VALUE (The filtered global position 
            (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). 
            It is designed as scaled integer message since the resolution of float is not sufficient.)


            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:

            {
                mavlink_global_position_int_t global_position_int;
                mavlink_msg_global_position_int_decode(&msg, &global_position_int);
#ifdef SOFT_SERIAL_DEBUGGING
                Serial.println("Vx: ");
                Serial.print(global_position_int.vx); //Ground X Speed (Latitude, positive north)
                Serial.println("Vz: ");
                Serial.print(global_position_int.vz); //Ground Y Speed (Longitude, positive east)
                Serial.println("Vy: ");
                Serial.print(global_position_int.vy); //Ground Z Speed (Altitude, positive down)
                Serial.println("Lat: ");
                Serial.print(global_position_int.lat); //Latitude, expressed
                Serial.println("Lon: ");
                Serial.print(global_position_int.lon); //Lon, expressed


#endif
            }
            break;
*/

//----------------------Gyro,Acc,Mag (x,y,z)------------------------------------------------------------------

//Rezultatas:




            /*
            case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
            {
                /* Message decoding: PRIMITIVE
                      static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
                
                mavlink_raw_imu_t raw_imu;
                mavlink_msg_raw_imu_decode(&msg, &raw_imu);

#ifdef SOFT_SERIAL_DEBUGGING

               // problema su sintakse, nesigauna isvesti

                Serial.print("X,Z,Y acc:");
                Serial.println(raw_imu.xacc, raw_imu.yacc, raw_imu.zacc);

               // Gerai gaunasi viskas tik noriu viska i viena eilute konsoleje
               
                Serial.print("X,Z,Y acc:");
                Serial.println(raw_imu.xacc);
                
                Serial.print("Y acc:");
                Serial.println(raw_imu.yacc);

                Serial.print("Z acc:");
                Serial.println(raw_imu.zacc);


                Serial.print("X gyro");
                Serial.println(raw_imu.xgyro);
                
                Serial.print("Y gyro");
                Serial.println(raw_imu.ygyro);

                Serial.print("Z gyro");
                Serial.println(raw_imu.zgyro);


                Serial.print("X mag:");
                Serial.println(raw_imu.xmag);
             
                Serial.print("Y mag:");
                Serial.println(raw_imu.ymag);
                             
                Serial.print("Z mag:");
                Serial.println(raw_imu.zmag);
                
#endif
            }
            break;


            */



            //------------------------------------------YAW,PITCH,ROLL (Atrodo to ko mums reikia)-------------------------------------------------------------------------------------------------
            //Rezultatas:
            //Yaw:0.52 -- Roll angle (-pi..+pi)
            //Pitch:-0.21 -- Pitch angle (-pi..+pi)
            //Roll:0.02 -- Yaw angle (-pi..+pi)
           /*
                       case MAVLINK_MSG_ID_ATTITUDE:  // #30


                       {
                           //mavlink_msg_attitude_decode(const mavlink_message_t * msg, mavlink_attitude_t * attitude);
                           mavlink_attitude_t attitude;
                           mavlink_msg_attitude_decode(&msg, &attitude);
           #ifdef SOFT_SERIAL_DEBUGGING
                           Serial.print("Pitch: ");
                           Serial.println(attitude.pitch);
                           Serial.print("Roll: ");
                           Serial.println(attitude.roll);
                           Serial.print("Yaw: ");
                           Serial.println(attitude.yaw);

           #endif
                       }
                       break;
           
           */

           //-----------------------------------------------------------GPS duomenys ir aukštis viskas (neveikia)----------------------------------------------------


                       // Rezultatas:
                       



                     //Sent from simulation to autopilot. 
                     //This packet is useful for high throughput applications such as hardware in the loop simulations.
          
                     case MAVLINK_MSG_ID_HIL_STATE:  // #90
                     {
                         //mavlink_msg_attitude_decode(const mavlink_message_t * msg, mavlink_attitude_t * attitude);
                         __mavlink_hil_state_t coordinates;
                         mavlink_msg_hil_state_decode(&msg, &coordinates);
         #ifdef SOFT_SERIAL_DEBUGGING
                         Serial.print("Alt: ");
                         Serial.println(coordinates.alt);
                         Serial.print("Lat: ");
                         Serial.println(coordinates.lat);
                         Serial.print("Lon: ");
                         Serial.println(coordinates.lon);
                         Serial.print("Roll: ");
                         Serial.println(coordinates.roll);
                         Serial.print("Pitch: ");
                         Serial.println(coordinates.pitch);
                         Serial.print("Yaw: ");
                         Serial.println(coordinates.yaw);



                        #endif
                     }
                     break;


         

         //--------------------------------------Kitas būdas gauti GPS (Veikia)----------------------------------------
                       //Rezultatas:
                       //Alt2:246370
                       //Lat2:54.7014308
                       //Lon2:25.2162280

/*


           case MAVLINK_MSG_ID_GPS_RAW_INT:  // #90
           {
               //mavlink_msg_attitude_decode(const mavlink_message_t * msg, mavlink_attitude_t * attitude);
               mavlink_gps_raw_int_t coordinate;
               mavlink_msg_gps_raw_int_decode(&msg, &coordinate);
#ifdef SOFT_SERIAL_DEBUGGING
                Serial.print("Alt2: ");
                Serial.println(coordinate.alt);
                Serial.print("Lat2: ");
                Serial.println(coordinate.lat);
                Serial.print("Lon2: ");
                Serial.println(coordinate.lon);
#endif
            }
            break;
            */

            }
        }
    }
    }