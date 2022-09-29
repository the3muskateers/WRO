 // Motor A

 int enA = 10;
 int in1 = 9;
 int in2 = 11;

 void setup()
 {
    // Set all the motor control pins to outputs
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

 } 
 void demoOne()
{
    //turn on motor A
 digitalWrite(in1, HIGH);
 digitalWrite(in2, LOW);
 // set speed to 200 out of possible range 0-255
 analougWrite(enA, 200);
 delay(2000);
 //now turn motor dierction
 digitalWrite(in1, LOW);
 digitalWrite(in2, HIGH);

 delay(2000);
 //now turn off motors
 digitalWrite(in1, LOW);
 digitalWrite(in2, LOW);

}
// void demoTwo()
// {
//     //this function will run the motors across the range of possible speeds
//     //note that maximum speed is  determined by the motor itself and the operating voltage

//     //turn on motors
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
    
// }