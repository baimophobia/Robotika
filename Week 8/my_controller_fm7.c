#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag motor_kanan = wb_robot_get_device("right wheel motor");
  WbDeviceTag motor_kiri = wb_robot_get_device("left wheel motor");
  WbDeviceTag sensor_kanan = wb_robot_get_device("right_sensor");
  WbDeviceTag sensor_kiri = wb_robot_get_device("left_sensor");
  
  wb_distance_sensor_enable(sensor_kanan, TIME_STEP);
  wb_distance_sensor_enable(sensor_kiri, TIME_STEP);
  
  wb_motor_set_position(motor_kanan, INFINITY);
  wb_motor_set_position(motor_kiri, INFINITY);
  
  double readIR_kanan, readIR_kiri;
  
  while (wb_robot_step(TIME_STEP) != -1) {
  
  readIR_kanan = wb_distance_sensor_get_value(sensor_kanan);
  readIR_kiri = wb_distance_sensor_get_value(sensor_kiri);
  
  printf("kanan 1 ");
  printf("%f/n",readIR_kanan);
  printf("kiri 1 ");
  printf("%f/n",readIR_kiri);
  
  
  if(readIR_kiri<readIR_kanan) {
     wb_motor_set_velocity(motor_kanan, 1);
     wb_motor_set_velocity(motor_kiri, 2.5);
  }
  
  if(readIR_kiri>readIR_kanan) {
     wb_motor_set_velocity(motor_kanan, 2.5);
     wb_motor_set_velocity(motor_kiri, 1);
     
  }
  
  };
  
  wb_robot_cleanup();

  return 0;
}