#include <SPI.h>
#include <mcp2515_can.h>
#include <MsTimer2.h> 

  /*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SerialUSB
#else
#define SERIAL Serial
#endif

//Value Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -15.0f
#define T_MAX 15.0f

// Set Values
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 0.0f;
float kd_in = 0.50f;
float t_in = 0.0f;

//measured values - responses from the motor
float p_out = 0.0f;  // actual position
float v_out = 0.0f;  // actual velocity
float t_out = 0.0f;  // actual torque


mcp2515_can CAN(10);  // Set CS pin
int CAN_INT = 2;

void setup() {

  SERIAL.begin(115200);
  while (!Serial) {};
  while (CAN_OK != CAN.begin(CAN_1000KBPS)) {  // init can bus : baudrate = 1000k
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }


  Zero();
  EnterMode();
  SERIAL.println("CAN BUS Shield init ok!");

}

void loop() {
  char rc;
  rc = Serial.read();
  Serial.println(rc);
  delay(500);

  float p_step = 0.1;
  
  if(rc == 'u')
  v_in = v_in + 3.0;

  if(rc == 'd')
  v_in = v_in - 3.0;

  p_in = constrain(p_in, P_MIN, P_MAX);

  if(rc == 's')
  EnterMode();

  if(rc == 'e')
  ExitMode();

  //send CAN
  pack_cmd();

  //receive CAN
  if(CAN_MSGAVAIL == CAN.checkReceive())
  {
   
    unpack_reply();
  }
  
  //print data
  Serial.print(" ");
  Serial.print(p_in);
  Serial.print(" ");
  Serial.print(p_out);
  Serial.print(" ");
  Serial.print(v_out);
  Serial.print(" ");
  Serial.println(t_out);
}

void EnterMode() {
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFC;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}
void ExitMode() {
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFD;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}
void Zero() {
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFE;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}
unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  ///Converts a  float to an unsigned int, given range and number of bits///
  float span = x_max-x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if(bits==12){
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  }
  if(bits==16){
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}
float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  ///converts unsigned int to float, given range and number of bits///
  float span = x_max-x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits==12){
    pgg = ((float) x_int)*span/4095 + offset;
  }
  if (bits==16){
    pgg = ((float) x_int)*span/65535.0 + offset;
  }
  return pgg;

}
void pack_cmd(){
  byte buf[8];
  ///CAN Command Packet Structure///
  ///16 bit position command, between -4*pi and 4*pi
  ///12 bit velocity command, between -30 and +30rad/s
  //12 bit kp, between 0 and 500 N-m/rad
  ///12 bit kd, between 0 and 100 N-m*s/rad
  /// 12 bit feed forward toque, between -18 and 18 N-m
  ///CAN packet is 8 8-bit words
  ///Formatted as follows. For each quantity, bit i0 is LSB
  ///0: [position[15-8[[
  ///1: [position [7-0]]
  ///2: [velocity[11-4]]
  ///3: [velocity[3-0], kp[11-8]]
  ///4: [kp[7-0]]
  ///5: [kd[11-4]]
  ///6:[kd[3-0], torque [11-8]]
  ///7: [torque [7-0]]

  ///limit data to be withing bounds///

  float p_des = constrain(p_in, P_MIN, P_MAX); ///fminf(fmaxf(P_MIN, p_in(, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX); ///fminf(fmaxf(V_MIN, v_in(, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX); ///fminf(fmaxf(KP_MIN, kp_in(, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX); ///fminf(fmaxf(KD_MIN, kd_in(, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX); ///fminf(fmaxf(T_MIN, t_in(, V_MAX);
  ///convert floats to unsigned ints///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  /// pack ints into the can buffer///
  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) <<4) | (kp_int >>8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >>4;
  buf[6] = ((kd_int & 0xF) <<4) | (t_int >>8);
  buf[7] = t_int & 0xFF;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}
void unpack_reply() {
  /// CAB reply Oacjet structure///
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and +30 rad/s
  /// 12 bit current, between -40 and 40;
  /// CAN Packet is 5 8-bit words
  ///Formatted as follows. For each quantity, bit 0 is LSB
  /// 0: [position [15-8]]
  /// 1: [position[7-0]]
  /// 2: [velocity [11-4]]
  /// 3: [velocity [3-0], current [11-8]]
  /// 4: [current [7-0]]

  byte len = 0;
  byte buf[8];
  unsigned long canId;

  CAN.readMsgBuf(&len, buf); 
  //read data
  //MCP_CAN::readMsgBuf(*len,  buf[])

  
  /// unpack ints from CAN buffer ///
  unsigned int id = buf[0];
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];
  /// convert uints to floats ///
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
}