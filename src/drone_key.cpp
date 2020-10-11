
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#define LIN_VEL_STEP_SIZE  0.01
#define ANG_VEL_STEP_SIZE  0.1
const char* msg = R"(
control Your drone!
---------------------------
Moving around:
   q    w    e
   a    s    d  
        x

w/x : increase/decrease linear velocity z
q/e : increase/decrease linear velocity x
a/d : increase/decrease angular velocity z
space key, s : keeping

CTRL-C to quit
)";

char keys =' ';

int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

void vels(double target_linear_vel_x,double target_linear_vel_z,double target_angular_vel){
    printf("\ncurrently:\nlinear vel_x %lf\n linear vel_z %lf\n angular vel %lf\n", target_linear_vel_x,target_linear_vel_z,target_angular_vel);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_teleop_key");
    ros::NodeHandle nh;

    ros::Publisher key_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);	

    geometry_msgs::Twist twist;

    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    
    double target_linear_vel_x = 0.0;
    double target_linear_vel_z = 0.0;
    double target_angular_vel = 0.0;
    double control_linear_vel_x  = 0.0;
    double control_linear_vel_z  = 0.0;
    double control_angular_vel = 0.0;

    printf("%s", msg);
    while(ros::ok()){

        keys = getch();
        printf("%c\n",    keys);
        if(keys == 'w'){
            target_linear_vel_z += LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys == 'x'){
            target_linear_vel_z -= LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys == 'a'){
            target_angular_vel += LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys == 'd'){
            target_angular_vel -= LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys == 'q'){
            target_linear_vel_x += LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys == 'e'){
            target_linear_vel_x -= LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys ==' ' or keys =='s'){
            target_linear_vel_x = 0.0;
            target_linear_vel_z = 0.0;
            target_angular_vel = 0.0;
            control_linear_vel_x  = 0.0;
            control_linear_vel_z = 0.0;
            control_angular_vel = 0.0;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        } else {
            if(keys == '\x03')
                break;
        }

        twist.linear.x = target_linear_vel_x;
        twist.linear.y = 0.0;
        twist.linear.z = target_linear_vel_z;

        twist.angular.x = 0.0;
        twist.angular.y = 0,0;
        twist.angular.z = target_angular_vel;
        key_pub.publish(twist);
        ros::spinOnce();
    }

    return 0;
}