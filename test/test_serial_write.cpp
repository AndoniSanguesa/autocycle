#include <ros/ros.h>
#include <serial/serial.h>

int main(int argc, char **argv){
    // Registers node with the master
    ros::init(argc, argv, "test_serial_write");
    ros::NodeHandle nh;

    // Creates serial object to write to
    serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

    // String to repeat (that will hopefully be echoed
    string to_repeat = "Hello World.\n";

    // Current index for the byte we are printing
    int ind = 0;

    // Prints string one byte at a time then receives the byte back and prints it
    while(ros::ok()){
        my_serial.print(to_repeat[ind%to_repeat.length()]);
        cout << my_serial.read();
        ind++;
    }
}