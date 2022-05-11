// Created by Shane Rozen-Levy on 9/21/19.

#include "kodlab_mjbots_sdk/PID.h"
#include <iostream>
#include <math.h>
PID::PID()
{
  m_kp=0;
  m_ki=0;
  m_kd=0;
  m_cum_error = 0;

  reset();
}

PID::PID(const float kp, const float ki, const float kd)
{
  m_kp = kp;
  m_ki = ki;
  m_kd = kd;

  reset();
}

void PID::reset()
{
  m_prev_sensor_val = 0;
  m_cum_error = 0;
  m_first_update = true;
}

void PID::calc_effort(const float target, const float sensor, float &effort)
{
  float error = target-sensor;
  m_cum_error = m_cum_error + error;
  float diff_error= sensor-m_prev_sensor_val;

  if(m_first_update){
    effort = m_kp*error + m_ki*m_cum_error;
    m_first_update = false;
  }
  else {
    effort = m_kp*error + m_ki*m_cum_error - m_kd*diff_error;
  }

  m_prev_sensor_val = sensor;
}

void PID::calc_effort(const float target, const float sensor, const float dsensor, float &effort)
{
  float error = target-sensor;
  m_cum_error = m_cum_error + m_ki * error;

  effort = m_kp*error + m_ki * m_cum_error - m_kd*dsensor;

  m_prev_sensor_val = sensor;
}

void PID::calc_effort(const float target, const float target_speed, const float sensor, const float dsensor, float &effort){
  float error = target-sensor;
  float speed_error = target_speed - dsensor;
  m_cum_error = m_cum_error + m_ki * error;

  effort = m_kp*error + m_ki * m_cum_error + m_kd*speed_error;

  m_prev_sensor_val = sensor;
}

