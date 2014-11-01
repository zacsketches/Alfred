//Data structures
#include <Vector.h>
#include <Pair.h>

//Glow Worm core
#include <clearinghouse.h>

//Glow Worm Messages
#include <messages/five_pt_scan.h>

//Glow Worm Blocks

//other libraries (Servo, Wire, etc)

//Logging Macros
#define LOG_UART Serial
#define LOG(x) LOG_UART.println(x)
#define LOG_P(x_float, prec) LOG_UART.println(x_float, prec); 

/*------------Required to initialize GW framework------------------------*/
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

/*------------Create a set of global messsages---------------------------*/
Five_pt_scan_msg five_pt_scan_msg;

/*------------Physical Connections--------------------------------------*/


/*------------Construct the system blocks-------------------------------*/

/*------------Setup-----------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  Serial.println();
  
  five_pt_scan_msg.print();
  
  gw::Scan_pt s1;
  Serial.println(text(s1));
  int h = s1.heading();
  int r = s1.range();
  Serial.println(h);
  Serial.println(r);
  
  delay(2000);
}

/*------------Loop------------------------------------------------------*/
void loop() {
  while(1);
}
