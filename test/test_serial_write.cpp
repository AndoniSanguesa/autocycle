#include <ros/ros.h>
#include <serial/serial.h>
#include <string>

using namespace std;


int main(int argc, char **argv){
    // Registers node with the master
    ros::init(argc, argv, "test_serial_write");
    ros::NodeHandle nh;

    // Creates serial object to write to
    serial::Serial my_serial("/dev/ttyACM0", (long) 1, serial::Timeout::simpleTimeout(0));

    // String to repeat (that will hopefully be echoed
    string to_repeat = "Hello World.\n";

    // Current index for the byte we are printing
    int ind = 0;
    uint8_t cur;
    string read;

    // Prints string one byte at a time then receives the byte back and prints it
    while(ros::ok()){
	cur = (uint8_t) to_repeat[ind%to_repeat.length()];
        my_serial.write(&cur, 1);
        while(read == ""){
	    read.append(my_serial.read());
        }
	cout << read << flush;
	read.clear();
        ind++;
    }
}
