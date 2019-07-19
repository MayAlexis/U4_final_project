/*
 * File:          v_1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>

#include <stdio.h>
#include <math.h>

/*
 * macros
 */
#define TIME_STEP 64
#define PI 3.141592

enum {
  Autonomous,
  Manual
};

int  G = 71, W = 87, A = 65, B = 66;
int state;
int initial_angle_motor1, angle, angle1, error, setpoint, Change_error,
Last_error, Total_error, pid_term, pid_term1, pid_term_scaled, kp =1.7, ki = 0.03, kd = 0.2,
angle_init, error_motor1, error_motor2;

/*
 * desire positiom of just positive values in radians
 */
int desiredposition = 2, desiredposition1 = 3, desiredposition3 = 3.14, desiredposition4 = 3.14;


/*
 * main
 */
 int controlPID(WbDeviceTag pos_sensor) {
  int angle_init = wb_position_sensor_get_value(pos_sensor);
  int angle;
  
  error = desiredposition - angle_init;
  angle = error - Last_error;
  Total_error += error;
  pid_term = (kp*error) + (ki*Total_error) + (kd*Change_error);
  pid_term = fabs(pid_term);
  Last_error = error;

  return pid_term;
}

 int controlPID1(WbDeviceTag pos_sensor) {
  int angle_init = wb_position_sensor_get_value(pos_sensor);
  int angle;
  
  error = desiredposition1 - angle_init;
  angle = error - Last_error;
  Total_error += error;
  pid_term = (kp*error) + (ki*Total_error) + (kd*Change_error);
  pid_term = fabs(pid_term);
  Last_error = error;

  return pid_term;
}




int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
  
  int key;
   
   WbDeviceTag motor[3];
     motor[0] = wb_robot_get_device("motorDC1");
     motor[1] = wb_robot_get_device("motorDC2");
     motor[2] = wb_robot_get_device("servoMotor1");
     motor[3] = wb_robot_get_device("servoMotor2");
  
    wb_motor_set_position(motor[0], INFINITY);
    wb_motor_set_position(motor[1], INFINITY);  
    wb_motor_set_position(motor[2], INFINITY);
    wb_motor_set_position(motor[3], INFINITY);
   
   WbDeviceTag encoder[1];
     encoder[0] = wb_robot_get_device("encoder1");
     encoder[1] = wb_robot_get_device("encoder2");
     
   wb_position_sensor_enable(encoder[0], TIME_STEP);
   wb_position_sensor_enable(encoder[1], TIME_STEP);
   
  
   

   
  /* 
   * main loop
   */
  while (wb_robot_step(TIME_STEP) != -1) {
  
  


     key = wb_keyboard_get_key();
     
     if (key == G)
       state = Autonomous;
     else if (key == W){
       state = Manual; 
       }
     if (state == Autonomous){
    

       
       pid_term = controlPID(encoder[1]);
       pid_term1 = controlPID1(encoder[0]);
       
      
       
       
       wb_motor_set_velocity(motor[1], pid_term);
       wb_motor_set_velocity(motor[0], pid_term1);
       wb_motor_set_position(motor[2], desiredposition3);
       wb_motor_set_position(motor[3], desiredposition4);
       
       
   

     
     } else {
         if (key == WB_KEYBOARD_UP){
           wb_motor_set_velocity(motor[0],0);
           wb_motor_set_velocity(motor[1],-3);
           wb_motor_set_velocity(motor[2],0);
           wb_motor_set_velocity(motor[3],0);
           angle = wb_position_sensor_get_value(encoder[0]);
           printf("Angle: %i\n", angle);
           angle = wb_position_sensor_get_value(encoder[1]);
           printf("Angle1: %i\n", angle);
         
         } else if (key == WB_KEYBOARD_DOWN){
           wb_motor_set_velocity(motor[1],0);
           wb_motor_set_velocity(motor[1],3);  
           wb_motor_set_velocity(motor[2],0);
           wb_motor_set_velocity(motor[3],0);
       
           
         } else if (key == WB_KEYBOARD_RIGHT){
           wb_motor_set_velocity(motor[0],-3);
           wb_motor_set_velocity(motor[1],0); 
           wb_motor_set_velocity(motor[2],0);
           wb_motor_set_velocity(motor[3],0);
        
              
         } else if (key == WB_KEYBOARD_LEFT){
           wb_motor_set_velocity(motor[0],3);
           wb_motor_set_velocity(motor[1],0);
           wb_motor_set_velocity(motor[2],0);
           wb_motor_set_velocity(motor[3],0); 
       
             
         }  else if (key == A) {
           wb_motor_set_velocity(motor[0],0);
           wb_motor_set_velocity(motor[1],0);
           wb_motor_set_velocity(motor[2],3);
           wb_motor_set_velocity(motor[3],0); 
             
         } else if (key == B) {
           wb_motor_set_velocity(motor[0],0);
           wb_motor_set_velocity(motor[1],0);
           wb_motor_set_velocity(motor[2],0);
           wb_motor_set_velocity(motor[3],3); 
             
         }else {
           wb_motor_set_velocity(motor[0],0);
           wb_motor_set_velocity(motor[1],0); 
           wb_motor_set_velocity(motor[2],0);
           wb_motor_set_velocity(motor[3],0);
               
         }
       }   
  }
  

  
  

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();


  return 0;
};
