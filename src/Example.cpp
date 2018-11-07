#include <iostream>   // cout()
#include <iomanip>    // setprecision() etc.
#include <stdexcept>  // runtime_error
#include <cstdio>     // getchar()
#include "RazorAHRS.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

#define pi 3.1415926

using namespace std;


// Set your serial port here!
//const string serial_port_name = "/dev/tty.FireFly-6162-SPP";
//const string serial_port_name = "/dev/tty.usbserial-A700eEhN";//改到下面这个了
const string serial_port_name = "/dev/ttyUSB0"; // a good guess on linux
std::string turtle_name;
ros::Publisher posepub;
visualization_msgs::Marker points, line_strip, line_list;

// Razor error callback handler
// Will be called from (and in) Razor background thread!
void on_error(const string &msg)
{
  cout << "  " << "ERROR: " << msg << endl;

  // NOTE: make a copy of the message if you want to save it or send it to another thread. Do not
  // save or pass the reference itself, it will not be valid after this function returns!
}

// Razor data callback handler
// Will be called from (and in) Razor background thread!
// 'data' depends on mode that was set when creating the RazorAHRS object. In this case 'data'
// holds 3 float values: yaw, pitch and roll.
void on_data(const float data[])
{
  cout << "  " << fixed << setprecision(1)
  << "Yaw = " << setw(6) << data[0] << "      Pitch = " << setw(6) << data[1] << "      Roll = " << setw(6) << data[2] << endl;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0.0) );
  tf::Quaternion q;
  q.setRPY(data[2]/180*pi, data[1]/180*pi, data[0]/180*pi);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "myimu"));
  // NOTE: make a copy of the yaw/pitch/roll data if you want to save it or send it to another
  // thread. Do not save or pass the pointer itself, it will not be valid after this function
  // returns!

  // If you created the Razor object using RazorAHRS::ACC_MAG_GYR_RAW or RazorAHRS::ACC_MAG_GYR_CALIBRATED
  // instead of RazorAHRS::YAW_PITCH_ROLL, 'data' would contain 9 values that could be printed like this:

  // cout << "  " << fixed << setprecision(1)
  // << "ACC = " << setw(6) << data[0] << ", " << setw(6) << data[1] << ", " << setw(6) << data[2]
  // << "        MAG = " << setw(7) << data[3] << ", " << setw(7) << data[4] << ", " << setw(7) << data[5]
  // << "        GYR = " << setw(7) << data[6] << ", " << setw(7) << data[7] << ", " << setw(7) << data[8] << endl;

}

RazorAHRS *razor;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "imushow");
  cout << endl;
  cout << "  " << "Razor AHRS C++ test" << endl;
  cout << "  " << "Press RETURN to connect to tracker. When you're done press RETURN again to quit." << endl;
  getchar();  // wait RETURN
  cout << "  " << "Connecting..." << endl << endl;

  try
  {
    // Create Razor AHRS object. Serial I/O will run in background thread and report
    // errors and data updates using the callbacks on_data() and on_error().
    // We want to receive yaw/pitch/roll data. If we wanted the unprocessed raw or calibrated sensor
    // data, we would pass RazorAHRS::ACC_MAG_GYR_RAW or RazorAHRS::ACC_MAG_GYR_CALIBRATED
    // instead of RazorAHRS::YAW_PITCH_ROLL.
    razor = new RazorAHRS(serial_port_name, on_data, on_error, RazorAHRS::YAW_PITCH_ROLL);

    // NOTE: If these callback functions were members of a class and not global
    // functions, you would have to bind them before passing. Like this:

    // class Callback
    // {
    //   public:
    //     void on_data(const float ypr[]) { }
    //     void on_error(const string &msg) { }
    // };

    // Callback c;

    // razor = new RazorAHRS(serial_port_name,
    //    bind(&Callback::on_data, &c, placeholders::_1),
    //    bind(&Callback::on_error, &c, placeholders::_1),
    //    RazorAHRS::YAW_PITCH_ROLL);

    // If you're calling from inside of "c" you would of course use "this" instead of "&c".
  }
  catch(runtime_error &e)
  {
    cout << "  " << (string("Could not create tracker: ") + string(e.what())) << endl;
    cout << "  " << "Did you set your serial port in Example.cpp?" << endl; // 报这个错是未将当前用户加入dialout用户组
    return 0;
  }

  getchar();  // wait for RETURN key
  return 0;
}
