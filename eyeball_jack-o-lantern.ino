/*************************************************************************************************
 * NunchuckPrint
 *
 * 2007 Tod E. Kurt, http://todbot.com/blog/
 *   
 * Change log:
 * 
 *      Mark Tashiro -  Changed Wire.read to Wire.write
 *                      Changed Wire.receive to Wire.read
 *                      Added code for servos
 *                      https://www.hackster.io/mtashiro/control-servos-using-wii-nunchuk-9136bd
 *
 *      Sam Pearce -    added more servos for second eyeball and eyelids
 *
 *
 **************************************************************************************************/
 
#include <Wire.h>
#include <Servo.h>
#include <Bounce2.h>

//create servos
Servo g_servo_left_x;
Servo g_servo_left_y;
Servo g_servo_left_lid;
Servo g_servo_right_x;
Servo g_servo_right_y;
Servo g_servo_right_lid;

//set servo pins
const int g_left_x_pin = 2;
const int g_left_y_pin = 3;
const int g_left_lid_pin = 4;
const int g_right_x_pin = 5;
const int g_right_y_pin = 6;
const int g_right_lid_pin = 7;

//set servo min and max movement
const int g_servo_left_x_min = 130;
const int g_servo_left_x_max = 60;
const int g_servo_left_y_min = 74;
const int g_servo_left_y_max = 118;
const int g_servo_left_lid_min = 40;
const int g_servo_left_lid_max = 140;
const int g_servo_right_x_min = 130;
const int g_servo_right_x_max = 60;
const int g_servo_right_y_min = 118;
const int g_servo_right_y_max = 74;
const int g_servo_right_lid_min = 40;
const int g_servo_right_lid_max = 140;

const int g_joy_min_x = 23;
const int g_joy_max_x = 222;
const int g_joy_min_y = 32;
const int g_joy_max_y = 231;

static uint8_t nunchuck_buf[6];   // array to store nunchuck data,

const int g_calibration_button = 9;
const int g_calibration_pot = A0;

void setup()
{
    Serial.begin(9600);

    //move servos to center position befor attaching
    g_servo_left_x.write((g_servo_left_x_min + g_servo_left_x_max) / 2);
    g_servo_left_y.write((g_servo_left_y_min + g_servo_left_y_max) / 2);
    g_servo_right_x.write((g_servo_left_lid_min + g_servo_left_lid_max) / 2);
    g_servo_right_y.write((g_servo_right_x_min + g_servo_right_x_max) / 2);
    g_servo_left_lid.write((g_servo_right_y_min + g_servo_right_y_max) / 2);
    g_servo_right_lid.write((g_servo_right_lid_min + g_servo_right_lid_max) / 2);

    //attach all servos
    g_servo_left_x.attach(g_left_x_pin);
    g_servo_left_y.attach(g_left_y_pin);
    g_servo_left_lid.attach(g_left_lid_pin);
    g_servo_right_x.attach(g_right_x_pin);
    g_servo_right_y.attach(g_right_y_pin);
    g_servo_right_lid.attach(g_right_lid_pin);
  
    nunchuck_setpowerpins(); // use analog pins 2&3 as fake gnd & pwr
    nunchuck_init(); // send the initilization handshake

    Serial.print ("Finished setup\n");

    pinMode(g_calibration_button, INPUT_PULLUP);
   // if (digitalRead(g_calibration_button) == 0)
    //while(1==1){
      //  calibrationReadings();
   // }
    
}

void loop()
{
    nunchuck_get_data();

    //constrain readings
    int x_constrained = constrain(nunchuck_buf[0], g_joy_min_x, g_joy_max_x);
    int y_constrained = constrain(nunchuck_buf[1], g_joy_min_y, g_joy_max_y);

    // map nunchuk data to a servo data point
    //int x_axis = map(x_constrained, g_joy_min_x, g_joy_max_x, 0, 180);
    //int y_axis = map(y_constrained, g_joy_min_y, g_joy_max_y, 0, 180);

    //move eyeballs & lids to desired position based on Wii nunchuk reading 
    g_servo_left_x.write(map(x_constrained, g_joy_min_x, g_joy_max_x, g_servo_left_x_min, g_servo_left_x_max));
    g_servo_left_y.write(map(y_constrained, g_joy_min_y, g_joy_max_y, g_servo_left_y_min, g_servo_left_y_max));
    g_servo_left_lid.write(map(y_constrained, g_joy_min_y, g_joy_max_y, g_servo_left_lid_min, g_servo_left_lid_max));
    g_servo_right_x.write(map(x_constrained, g_joy_min_x, g_joy_max_x, g_servo_right_x_min, g_servo_right_x_max));
    g_servo_right_y.write(map(y_constrained, g_joy_min_y, g_joy_max_y, g_servo_right_y_min, g_servo_right_y_max));
    g_servo_right_lid.write(map(y_constrained, g_joy_min_y, g_joy_max_y, g_servo_right_lid_min, g_servo_right_lid_max));
    
    // un-comment next line to print data to serial monitor  
    nunchuck_print_data();

}


//
// Nunchuck functions
//

// Uses port C (analog in) pins as power & ground for Nunchuck
static void nunchuck_setpowerpins()
{
#define pwrpin PORTC3
#define gndpin PORTC2
    DDRC |= _BV(pwrpin) | _BV(gndpin);
    PORTC &=~ _BV(gndpin);
    PORTC |=  _BV(pwrpin);
    delay(100);  // wait for things to stabilize        
}

// initialize the I2C system, join the I2C bus,
// and tell the nunchuck we're talking to it
void nunchuck_init()
{ 
    Wire.begin();                      // join i2c bus as master
    Wire.beginTransmission(0x52);     // transmit to device 0x52
    Wire.write(0x40);            // sends memory address
    Wire.write(0x00);            // sends sent a zero.  
    Wire.endTransmission();     // stop transmitting
}

// Send a request for data to the nunchuck
// was "send_zero()"
void nunchuck_send_request()
{
    Wire.beginTransmission(0x52);     // transmit to device 0x52
    Wire.write(0x00);            // sends one byte
    Wire.endTransmission();     // stop transmitting
}

// Receive data back from the nunchuck, 
int nunchuck_get_data()
{
    int cnt=0;
    Wire.requestFrom (0x52, 6);     // request data from nunchuck
    while (Wire.available ())
    {
        // receive byte as an integer
        nunchuck_buf[cnt] = nunchuk_decode_byte(Wire.read());
        cnt++;
    }
    nunchuck_send_request();  // send request for next data payload
    // If we recieved the 6 bytes, then go print them
    if (cnt >= 5) 
    {
        return 1;   // success
    }
    return 0; //failure
}

// Print the input data we have recieved
// accel data is 10 bits long
// so we read 8 bits, then we have to add
// on the last 2 bits.  That is why I
// multiply them by 2 * 2
void nunchuck_print_data()
{ 
    static int i=0;
    int joy_x_axis = nunchuck_buf[0];
    int joy_y_axis = nunchuck_buf[1];

    int accel_x_axis = nunchuck_buf[2]; // * 2 * 2; 
    int accel_y_axis = nunchuck_buf[3]; // * 2 * 2;
    int accel_z_axis = nunchuck_buf[4]; // * 2 * 2;


    int z_button = 0;
    int c_button = 0;

    // byte nunchuck_buf[5] contains bits for z and c buttons
    // it also contains the least significant bits for the accelerometer data
    // so we have to check each bit of byte outbuf[5]
    if ((nunchuck_buf[5] >> 0) & 1) 
        z_button = 1;
    if ((nunchuck_buf[5] >> 1) & 1)
        c_button = 1;

    if ((nunchuck_buf[5] >> 2) & 1) 
        accel_x_axis += 2;
    if ((nunchuck_buf[5] >> 3) & 1)
        accel_x_axis += 1;

    if ((nunchuck_buf[5] >> 4) & 1)
        accel_y_axis += 2;
    if ((nunchuck_buf[5] >> 5) & 1)
        accel_y_axis += 1;

    if ((nunchuck_buf[5] >> 6) & 1)
        accel_z_axis += 2;
    if ((nunchuck_buf[5] >> 7) & 1)
        accel_z_axis += 1;

    Serial.print(i,DEC);
    Serial.print("\t");
  
    Serial.print("joy:");
    Serial.print(joy_x_axis,DEC);
    Serial.print(",");
    Serial.print(joy_y_axis, DEC);
    Serial.print("  \t");

    Serial.print("acc:");
    Serial.print(accel_x_axis, DEC);
    Serial.print(",");
    Serial.print(accel_y_axis, DEC);
    Serial.print(",");
    Serial.print(accel_z_axis, DEC);
    Serial.print("\t");

    Serial.print("but:");
    Serial.print(z_button, DEC);
    Serial.print(",");
    Serial.print(c_button, DEC);

    Serial.print("\r\n");  // newline
    i++;
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
char nunchuk_decode_byte (char x)
{
    x = (x ^ 0x17) + 0x17;
    return x;
}

void calibrationReadings()
{
    
    nunchuck_get_data();   
    static int pot_position = -1;
    static int current_servo = 0;
    static int servo[6];
    while((g_calibration_button == 0) && (pot_position == -1)){
        pot_position = 0;
    }
    nunchuck_print_data();
    current_servo = (current_servo + was_button_pushed(&g_calibration_button)) % 6;
    pot_position = analogRead(g_calibration_pot);
    int servo_position = map(pot_position, 0, 1023, 10, 170);
    
    switch(current_servo)
    {
        case 0:
            g_servo_left_x.write(servo[0] = servo_position);
            break;
        case 1:
            g_servo_left_y.write(servo[1] = servo_position);
            break;
        case 2:
            g_servo_left_lid.write(servo[2] = servo_position);
            break;
        case 3:
            g_servo_right_y.write(servo[3] = servo_position);
            break;
        case 4:
            g_servo_right_x.write(servo[4] = servo_position);            
            break;
        case 5:
            g_servo_right_lid.write(servo[5] = servo_position);
            break;
    
    }
    Serial.print("Selected: ");
    Serial.print(current_servo + 1);
    Serial.print(" 1: ");
    Serial.print(servo[0]+1);
    Serial.print(" 2: ");
    Serial.print(servo[1]+1);
    Serial.print(" 3: ");
    Serial.print(servo[2]+1);
    Serial.print(" 4: ");
    Serial.print(servo[3]+1);
    Serial.print(" 5: ");
    Serial.print(servo[4]+1);
    Serial.print(" 6: ");
    Serial.print(servo[5]+1);
}

bool was_button_pushed(int *buttonPin)
{
    static int buttonState;             
    static int lastButtonState = LOW;   
    static unsigned long lastDebounceTime = 0;
    static unsigned long debounceDelay = 50;
    // read the state of the switch into a local variable:
    int reading = digitalRead(*buttonPin);
    bool total = 0;
    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState)
    {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:

        // if the button state has changed:
        if (reading != buttonState) 
        {
            buttonState = reading;

            // only toggle the change if the new button state is HIGH
            if (buttonState == HIGH)
            {
                total = 1;
            }
        }
    }
    // save the reading. Next time through the loop, it'll be the lastButtonState:
    lastButtonState = reading;
    return total;
}
