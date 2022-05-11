// Created by Shane Rozen-Levy on 9/21/19.

#ifndef KODLAB_WS_SRC_JERBOA_CONTROL_INCLUDE_JERBOA_CONTROL_PID_H_
#define KODLAB_WS_SRC_JERBOA_CONTROL_INCLUDE_JERBOA_CONTROL_PID_H_
class PID
{
private:

  float m_kp = 0; ///< proportional gain for PID loop
  float m_ki = 0; ///< integral gain for PID loop
  float m_kd = 0; ///< derivative gain for PID loop

  float m_prev_sensor_val = 0; ///< the previous sensor value, used in derivative
  float m_cum_error  = 0;      ///< the cumulation of the error, used in integral
  bool m_first_update = true;

public:
  /*!
   * @brief default constructor. Initializes everything to zero
   */
  PID();

  /*!
   * @brief constructor for PID object
   * @param kp[in] p gain
   * @param ki[in] i gain
   * @param kd[in] d gain
   */
  PID(const float kp, const float ki, const float kd);

  /*!
   * @brief resets the integrator and derivative term
   */
  void reset();

  /*!
   * @brief computes the effort based on the pid loop
   * @param target[in] the target value
   * @param sensor[in] the sensor value
   * @param effort[out] the effort to apply
   */
  void calc_effort(const float target, const float sensor, float &effort);

  /*!
 * @brief computes the effort based on the pid loop
 * @param target[in] the target value
 * @param sensor[in] the sensor value
 * @param dsensor[in] the derivative of the sensor value
 * @param effort[out] the effort to apply
 */
  void calc_effort(const float target, const float sensor, const float dsensor, float &effort);

 /*!
  * @brief computes the effort based on the pid loop
  * @param target[in] the target value
  * @param target_speed[in] the target speed
  * @param sensor[in] the sensor value
  * @param dsensor[in] the derivative of the sensor value
  * @param effort[out] the effort to apply
  */
  void calc_effort(const float target, const float target_speed, const float sensor, const float dsensor, float &effort);

};
#endif //KODLAB_WS_SRC_JERBOA_CONTROL_INCLUDE_JERBOA_CONTROL_PID_H_