#include "HFIBLDCMotor.h"
#include "./communication/SimpleFOCDebug.h"
#define LOWPASS( output, input, c_lowpass)  (output += (c_lowpass) * ((input) - (output)))

#ifndef SWAP_HILO
  #define SWAP_HILO true // true for ESP32
#endif
extern int** trap_120_map;
extern int** trap_150_map;


static inline float IRAM_ATTR _hfinormalizeAngle(float angle) {
	while (angle < 0) { angle += _2PI; }
	while (angle >=  _2PI) { angle -= _2PI; }
  return angle;
}

// HFIBLDCMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
// - KV            - motor kv rating (rmp/v)
// - L             - motor phase inductance
HFIBLDCMotor::HFIBLDCMotor(int pp, float _R, float _KV, float _inductance)
: FOCMotor()
{
  // save pole pairs number
  pole_pairs = pp;
  // save phase resistance number
  phase_resistance = _R;
  // save back emf constant KV = 1/KV
  // 1/sqrt(2) - rms value
  KV_rating = NOT_SET;
  if (_isset(_KV))
    KV_rating = _KV;
  // save phase inductance
  phase_inductance = _inductance;

  // torque control type is voltage by default
  torque_controller = TorqueControlType::voltage;
}


/**
	Link the driver which controls the motor
*/
void HFIBLDCMotor::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}

// init hardware pins
void HFIBLDCMotor::init() {
  if (!driver || !driver->initialized) {
    motor_status = FOCMotorStatus::motor_init_failed;
    SIMPLEFOC_DEBUG("MOT: Init not possible, driver not initialized");
    return;
  }
  motor_status = FOCMotorStatus::motor_initializing;
  SIMPLEFOC_DEBUG("MOT: Init");

  // sanity check for the voltage limit configuration
  if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
  // constrain voltage for sensor alignment
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

  // update the controller limits
  if(current_sense){
    // current control loop controls voltage
    PID_current_q.limit = voltage_limit;
    PID_current_d.limit = voltage_limit;

    PID_current_d.P = Ld*current_bandwidth*_2PI;
    PID_current_d.I = PID_current_d.P*(phase_resistance/2)/(Ld);
    PID_current_q.D = 0;
    PID_current_d.output_ramp = 0;

    PID_current_q.P = Lq*current_bandwidth*_2PI;
    PID_current_q.I = PID_current_q.P*(phase_resistance/2)/(Lq);
    PID_current_q.D = 0;
    PID_current_q.output_ramp = 0;

  }
  if(_isset(phase_resistance) || torque_controller != TorqueControlType::voltage){
    // velocity control loop controls current
    PID_velocity.limit = current_limit;
  }else{
    // velocity control loop controls the voltage
    PID_velocity.limit = voltage_limit;
  }
  P_angle.limit = velocity_limit;

  // if using open loop control, set a CW as the default direction if not already set
  if ((controller==MotionControlType::angle_openloop
     ||controller==MotionControlType::velocity_openloop)
     && (sensor_direction == Direction::UNKNOWN)) {
      sensor_direction = Direction::CW;
  }

  _delay(500);
  // enable motor
  SIMPLEFOC_DEBUG("MOT: Enable driver.");
  enable();
  _delay(500);
  motor_status = FOCMotorStatus::motor_uncalibrated;
}


// disable motor driver
void HFIBLDCMotor::disable()
{
  // disable the current sense
  if(current_sense) current_sense->disable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // disable the driver
  driver->disable();
  // motor status update
  enabled = 0;
}
// enable motor driver
void HFIBLDCMotor::enable()
{
  // enable the driver
  driver->enable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // enable the current sense
  if(current_sense) current_sense->enable();
  // reset the pids
  PID_velocity.reset();
  P_angle.reset();
  PID_current_q.reset();
  PID_current_d.reset();
  // motor status update
  enabled = 1;
}

/**
  FOC functions
*/
// FOC initialization function
int  HFIBLDCMotor::initFOC() {
  int exit_flag = 1;

  #ifdef HFI_2XPWM
    Ts = 1.0f/(2.0f*(float)driver->pwm_frequency);
  #else
    Ts = 1.0f/((float)driver->pwm_frequency);
  #endif
  Ts_L = 2.0f*Ts * ( 1 / Lq - 1 / Ld );
  motor_status = FOCMotorStatus::motor_calibrating;

  // align motor if necessary
  // alignment necessary for encoders!
  // sensor and motor alignment - can be skipped
  // by setting motor.sensor_direction and motor.zero_electric_angle
  if(sensor){
    exit_flag *= alignSensor();
    // added the shaft_angle update
    sensor->update();
    shaft_angle = shaftAngle();
  }else {
    // exit_flag = 0; // no FOC without sensor
    SIMPLEFOC_DEBUG("MOT: No sensor.");
  }

  // aligning the current sensor - can be skipped
  // checks if driver phases are the same as current sense phases
  // and checks the direction of measuremnt.
  if(exit_flag){
    if(current_sense){ 
      if (!current_sense->initialized) {
        motor_status = FOCMotorStatus::motor_calib_failed;
        SIMPLEFOC_DEBUG("MOT: Init FOC error, current sense not initialized");
        exit_flag = 0;
      }else{
        exit_flag *= alignCurrentSense();
      }
    }
    else { SIMPLEFOC_DEBUG("MOT: No current sense."); }
  }

  if(exit_flag){
    SIMPLEFOC_DEBUG("MOT: Ready.");
    motor_status = FOCMotorStatus::motor_ready;
  }else{
    SIMPLEFOC_DEBUG("MOT: Init FOC failed.");
    motor_status = FOCMotorStatus::motor_calib_failed;
    disable();
  }

  return exit_flag;
}

// Calibarthe the motor and current sense phases
int HFIBLDCMotor::alignCurrentSense() {
  int exit_flag = 1; // success

  SIMPLEFOC_DEBUG("MOT: Align current sense.");

  // align current sense and the driver
  exit_flag = current_sense->driverAlign(voltage_sensor_align);
  if(!exit_flag){
    // error in current sense - phase either not measured or bad connection
    SIMPLEFOC_DEBUG("MOT: Align error!");
    exit_flag = 0;
  }else{
    // output the alignment status flag
    SIMPLEFOC_DEBUG("MOT: Success: ", exit_flag);
  }

  return exit_flag > 0;
}

// Encoder alignment to electrical 0 angle
int HFIBLDCMotor::alignSensor() {
  int exit_flag = 1; //success
  SIMPLEFOC_DEBUG("MOT: Align sensor.");

  // check if sensor needs zero search
  if(sensor->needsSearch()) exit_flag = absoluteZeroSearch();
  // stop init if not found index
  if(!exit_flag) return exit_flag;

  // if unknown natural direction
  if(sensor_direction==Direction::UNKNOWN){

    // find natural direction
    // move one electrical revolution forward
    for (int i = 0; i <=500; i++ ) {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_sensor_align, 0,  angle);
	    sensor->update();
      _delay(2);
    }
    // take and angle in the middle
    sensor->update();
    float mid_angle = sensor->getAngle();
    // move one electrical revolution backwards
    for (int i = 500; i >=0; i-- ) {
      float angle = _3PI_2 + _2PI * i / 500.0f ;
      setPhaseVoltage(voltage_sensor_align, 0,  angle);
	    sensor->update();
      _delay(2);
    }
    sensor->update();
    float end_angle = sensor->getAngle();
    setPhaseVoltage(0, 0, 0);
    _delay(200);
    // determine the direction the sensor moved
    float moved =  fabs(mid_angle - end_angle);
    if (moved<MIN_ANGLE_DETECT_MOVEMENT) { // minimum angle to detect movement
      SIMPLEFOC_DEBUG("MOT: Failed to notice movement");
      return 0; // failed calibration
    } else if (mid_angle < end_angle) {
      SIMPLEFOC_DEBUG("MOT: sensor_direction==CCW");
      sensor_direction = Direction::CCW;
    } else{
      SIMPLEFOC_DEBUG("MOT: sensor_direction==CW");
      sensor_direction = Direction::CW;
    }
    // check pole pair number
    pp_check_result = !(fabs(moved*pole_pairs - _2PI) > 0.5f); // 0.5f is arbitrary number it can be lower or higher!
    if( pp_check_result==false ) {
      SIMPLEFOC_DEBUG("MOT: PP check: fail - estimated pp: ", _2PI/moved);
    } else {
      SIMPLEFOC_DEBUG("MOT: PP check: OK!");
    }

  } else { SIMPLEFOC_DEBUG("MOT: Skip dir calib."); }

  // zero electric angle not known
  if(!_isset(zero_electric_angle)){
    // align the electrical phases of the motor and sensor
    // set angle -90(270 = 3PI/2) degrees
    setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);
    _delay(700);
    // read the sensor
    sensor->update();
    // get the current zero electric angle
    zero_electric_angle = 0;
    zero_electric_angle = electricalAngle();
    //zero_electric_angle =  _normalizeAngle(_electricalAngle(sensor_direction*sensor->getAngle(), pole_pairs));
    _delay(20);
    if(monitor_port){
      SIMPLEFOC_DEBUG("MOT: Zero elec. angle: ", zero_electric_angle);
    }
    // stop everything
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  } else { SIMPLEFOC_DEBUG("MOT: Skip offset calib."); }
  return exit_flag;
}

// Encoder alignment the absolute zero angle
// - to the index
int HFIBLDCMotor::absoluteZeroSearch() {
  // sensor precision: this is all ok, as the search happens near the 0-angle, where the precision
  //                    of float is sufficient.
  SIMPLEFOC_DEBUG("MOT: Index search...");
  // search the absolute zero with small velocity
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while(sensor->needsSearch() && shaft_angle < _2PI){
    angleOpenloop(1.5f*_2PI);
    // call important for some sensors not to loose count
    // not needed for the search
    sensor->update();
  }
  // disable motor
  setPhaseVoltage(0, 0, 0);
  // reinit the limits
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;
  // check if the zero found
  if(monitor_port){
    if(sensor->needsSearch()) { SIMPLEFOC_DEBUG("MOT: Error: Not found!"); }
    else { SIMPLEFOC_DEBUG("MOT: Success!"); }
  }
  return !sensor->needsSearch();
}

void IRAM_ATTR HFIBLDCMotor::process_hfi(){
  // digitalToggle(PC10);
  // digitalToggle(PC10);

  // if hfi off, handle in normal way
  if (hfi_on == false || enabled==0) {
    hfi_firstcycle=true;
    hfi_int=0;
    hfi_out=0;
    hfi_full_turns=0;
    return;
  }

  #ifndef HFI_2XPWM
  bool is_v0 = driver->getPwmState();

  if (!is_v0) {
    driver->setPwm(Ua, Ub, Uc);
    // digitalToggle(PC10);
    // digitalToggle(PC10);
    return;
  }
  #endif

  float center;
  DQVoltage_s voltage_pid;
  DQCurrent_s current_err;
  float _ca,_sa;
  float hfi_v_act;
  _sincos(electrical_angle, &_sa, &_ca);
  // _sincos(1.5f, &_sa, &_ca);

  PhaseCurrent_s phase_current = current_sense->getPhaseCurrents();
  ABCurrent_s ABcurrent = current_sense->getABCurrents(phase_current);
  current_meas.d = ABcurrent.alpha * _ca + ABcurrent.beta * _sa;
  current_meas.q = ABcurrent.beta * _ca - ABcurrent.alpha * _sa;

  hfi_v_act = hfi_v;
  
  if (hfi_firstcycle) {
    hfi_v_act /= 2.0f;
    hfi_firstcycle=false;
  }

 #if SWAP_HILO == true 
    if (hfi_high) {
      current_low = current_meas;
    } else {
      current_high = current_meas;
      hfi_v_act = -1.0f*hfi_v;
    }
  #else
    if (hfi_high) {
      current_high = current_meas;
    } else {
      current_low = current_meas;
      hfi_v_act = -1.0f*hfi_v;
    }
  #endif

  hfi_high = !hfi_high;

  delta_current.q = current_high.q - current_low.q;
  delta_current.d = current_high.d - current_low.d;

  if (last_hfi_v != hfi_v || last_Ts != Ts || last_Ld != Ld || last_Lq != Lq)
  {
    predivAngleest = 1.0f / (hfi_v * Ts * ( 1.0f / Lq - 1.0f / Ld ) );
    last_hfi_v = hfi_v;
    last_Ts = Ts;
    last_Ld = Ld;
    last_Lq = Lq;
    Ts_div = 1.0f / Ts;
  }
  
  // hfi_curangleest = delta_current.q / (hfi_v * Ts_L );  // this is about half a us faster than vv
  
  // hfi_curangleest =  0.5f * delta_current.q / (hfi_v * Ts * ( 1.0f / Lq - 1.0f / Ld ) );
  hfi_curangleest =  0.5f * delta_current.q * predivAngleest;

  // LOWPASS(hfi_curangleest, 0.5f * delta_current.q / (hfi_v * Ts * ( 1.0f / Lq - 1.0f / Ld ) ), 0.34f);
  hfi_error = -hfi_curangleest;
  hfi_int += Ts * hfi_error * hfi_gain2; //This the the double integrator
  hfi_int = _constrain(hfi_int,-Ts*120.0f, Ts*120.0f);
  hfi_out += hfi_gain1 * Ts * hfi_error + hfi_int; //This is the integrator and the double integrator

  current_err.q = current_setpoint.q - current_meas.q;
  current_err.d = current_setpoint.d - current_meas.d;

  voltage_pid.q = PID_current_q(current_err.q, Ts, Ts_div);
  voltage_pid.d = PID_current_d(current_err.d, Ts, Ts_div);

  // lowpass does a += on the first arg
  LOWPASS(voltage.q,voltage_pid.q, 0.34f);
  LOWPASS(voltage.d,voltage_pid.d, 0.34f);

  voltage.d = _constrain(voltage.d ,-voltage_limit, voltage_limit);
  voltage.q = _constrain(voltage.q ,-voltage_limit, voltage_limit);
  voltage.d += hfi_v_act;
 
  // // PMSM decoupling control and BEMF FF
  // stateX->VqFF = stateX->we * ( confX->Ld * stateX->Id_SP + confX->Lambda_m);
  // stateX->VqFF += stateX->Iq_SP * stateX->R ;

  // // q axis induction FFW based on setpoint FFW
  // stateX->VqFF += stateX->jerk * stateX->Jload * confX->Lq / (1.5 * confX->N_pp * confX->Lambda_m) * stateX->OutputOn;

  // stateX->Vq += stateX->VqFF;

  // stateX->VdFF = -stateX->we * confX->Lq * stateX->Iq_SP;
  // stateX->Vd += stateX->VdFF;

  // setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
    // Inverse park transform

  

  Ualpha =  _ca * voltage.d - _sa * voltage.q;  // -sin(angle) * Uq;
  Ubeta =  _sa * voltage.d + _ca * voltage.q;    //  cos(angle) * Uq;

  // Clarke transform
  Ua = Ualpha;
  Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
  Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

  center = driver->voltage_limit/2.0f;
  float Umin = min(Ua, min(Ub, Uc));
  float Umax = max(Ua, max(Ub, Uc));
  center -= (Umax+Umin) / 2.0f;
  
  Ua += center;
  Ub += center;
  Uc += center;
  // Serial.printf(">hfiV:%f\n", Ua);
  #ifdef HFI_2XPWM
    // for hfi at 2x pwm
    driver->setPwm(Ua, Ub, Uc);
  #endif

  while (hfi_out < 0) { hfi_out += _2PI;}
	while (hfi_out >=  _2PI) { hfi_out -= _2PI;}
  hfi_int = _hfinormalizeAngle(hfi_int);

  float d_angle = hfi_out - electrical_angle;
  if(abs(d_angle) > (0.8f*_2PI) ) hfi_full_turns += ( d_angle > 0.0f ) ? -1.0f : 1.0f; 
  electrical_angle = hfi_out;
  // digitalToggle(PC10);
  // digitalToggle(PC10);  
  // digitalToggle(PC10);
  // digitalToggle(PC10);
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void HFIBLDCMotor::loopFOC() {
  // if hfi is on, we'll handle everything elsewhere
  if (hfi_on == true) {
    return;
  }
  // update sensor - do this even in open-loop mode, as user may be switching between modes and we could lose track
  //                 of full rotations otherwise.
  if (sensor) sensor->update();

  // if open-loop do nothing
  if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) return;
  
  // if disabled do nothing
  if(!enabled) return;

  // Needs the update() to be called first
  // This function will not have numerical issues because it uses Sensor::getMechanicalAngle() 
  // which is in range 0-2PI
  electrical_angle = electricalAngle();
  switch (torque_controller) {
    case TorqueControlType::voltage:
      // no need to do anything really
      break;
    case TorqueControlType::dc_current:
      if(!current_sense) return;
      // read overall current magnitude
      current.q = current_sense->getDCCurrent(electrical_angle);
      // filter the value values
      current.q = LPF_current_q(current.q);
      // calculate the phase voltage
      voltage.q = PID_current_q(current_setpoint.q - current.q);
      // d voltage  - lag compensation
      if(_isset(phase_inductance)) voltage.d = _constrain( -current_setpoint.q*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      else voltage.d = 0;
      break;
    case TorqueControlType::foc_current:
      if(!current_sense) return;
      // read dq currents
      current = current_sense->getFOCCurrents(electrical_angle);
      // filter values
      current.q = LPF_current_q(current.q);
      current.d = LPF_current_d(current.d);
      // calculate the phase voltages
      voltage.q = PID_current_q(current_setpoint.q - current.q);
      voltage.d = PID_current_d(-current.d);
      // d voltage - lag compensation - TODO verify
      // if(_isset(phase_inductance)) voltage.d = _constrain( voltage.d - current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      break;
    default:
      // no torque control selected
      SIMPLEFOC_DEBUG("MOT: no torque control selected!");
      break;
  }

  // set the phase voltage - FOC heart function :)
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or torque loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void HFIBLDCMotor::move(float new_target) {

  // downsampling (optional)
  if(motion_cnt++ < motion_downsample) return;
  motion_cnt = 0;

  // shaft angle/velocity need the update() to be called first
  // get shaft angle
  // TODO sensor precision: the shaft_angle actually stores the complete position, including full rotations, as a float
  //                        For this reason it is NOT precise when the angles become large.
  //                        Additionally, the way LPF works on angle is a precision issue, and the angle-LPF is a problem
  //                        when switching to a 2-component representation.
  if( controller!=MotionControlType::angle_openloop && controller!=MotionControlType::velocity_openloop ) ;
  if (hfi_on==true) {
    noInterrupts();
    float tmp_electrical_angle = electrical_angle;
    interrupts();
    float temp_shaft_angle = (hfi_full_turns *_2PI + tmp_electrical_angle)/pole_pairs;
    unsigned long currentUpdateTime = _micros();
    shaft_velocity = LPF_velocity(((temp_shaft_angle - shaft_angle)*1000000.0f/(currentUpdateTime - lastUpdateTime))/1);
    lastUpdateTime = currentUpdateTime;
    shaft_angle = temp_shaft_angle;
  } else {
    if (!sensor){
      shaft_angle = shaftAngle(); // read value even if motor is disabled to keep the monitoring updated but not in openloop mode
      // get angular velocity  TODO the velocity reading probably also shouldn't happen in open loop modes?
      shaft_velocity = shaftVelocity(); // read value even if motor is disabled to keep the monitoring updated
    }
  }
  // if disabled do nothing
  if(!enabled) return;
  // set internal target variable
  if(_isset(new_target)) target = new_target;
  
  // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
  if (_isset(KV_rating)) voltage_bemf = shaft_velocity/(KV_rating*_SQRT3)/_RPM_TO_RADS;
  // estimate the motor current if phase reistance available and current_sense not available
  if(!current_sense && _isset(phase_resistance)) current.q = (voltage.q - voltage_bemf)/phase_resistance;
  
  float temp_q_setpoint;
  // upgrade the current based voltage limit
  switch (controller) {
    case MotionControlType::torque:
      if(torque_controller == TorqueControlType::voltage){ // if voltage torque control
        if(!_isset(phase_resistance))  voltage.q = target;
        else  voltage.q =  target*phase_resistance + voltage_bemf;
        voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
        // set d-component (lag compensation if known inductance)
        if(!_isset(phase_inductance)) voltage.d = 0;
        else voltage.d = _constrain( -target*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      }else{
        noInterrupts();
        current_setpoint.q = target; // if current/foc_current torque control
        interrupts();
      }
      break;
    case MotionControlType::angle:
      // TODO sensor precision: this calculation is not numerically precise. The target value cannot express precise positions when
      //                        the angles are large. This results in not being able to command small changes at high position values.
      //                        to solve this, the delta-angle has to be calculated in a numerically precise way.
      // angle set point
      shaft_angle_sp = target;
      // calculate velocity set point
      temp_q_setpoint = feed_forward_velocity + P_angle( shaft_angle_sp - shaft_angle );
      temp_q_setpoint = _constrain(temp_q_setpoint,-current_limit, current_limit);
      // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
      noInterrupts();
      current_setpoint.q = temp_q_setpoint;
      interrupts();
      // if torque controlled through voltage
      if(torque_controller == TorqueControlType::voltage){
        // use voltage if phase-resistance not provided
        if(!_isset(phase_resistance))  voltage.q = current_setpoint.q;
        else  voltage.q =  _constrain( current_setpoint.q*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
        // set d-component (lag compensation if known inductance)
        if(!_isset(phase_inductance)) voltage.d = 0;
        else voltage.d = _constrain( -current_setpoint.q*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      }
      break;
    case MotionControlType::velocity:
      // velocity set point - sensor precision: this calculation is numerically precise.
      shaft_velocity_sp = target;
      // calculate the torque command
      temp_q_setpoint = PID_velocity(shaft_velocity_sp - shaft_velocity); // if current/foc_current torque control
      temp_q_setpoint = _constrain(temp_q_setpoint,-current_limit, current_limit);
      noInterrupts();
      current_setpoint.q = temp_q_setpoint;
      interrupts();
      // if torque controlled through voltage control
      if(torque_controller == TorqueControlType::voltage){
        // use voltage if phase-resistance not provided
        if(!_isset(phase_resistance))  voltage.q = current_setpoint.q;
        else  voltage.q = _constrain( current_setpoint.q*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
        // set d-component (lag compensation if known inductance)
        if(!_isset(phase_inductance)) voltage.d = 0;
        else voltage.d = _constrain( -current_setpoint.q*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      }
      break;
    case MotionControlType::velocity_openloop:
      // velocity control in open loop - sensor precision: this calculation is numerically precise.
      shaft_velocity_sp = target;
      voltage.q = velocityOpenloop(shaft_velocity_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
      break;
    case MotionControlType::angle_openloop:
      // angle control in open loop - 
      // TODO sensor precision: this calculation NOT numerically precise, and subject
      //                        to the same problems in small set-point changes at high angles 
      //                        as the closed loop version.
      shaft_angle_sp = target;
      voltage.q = angleOpenloop(shaft_angle_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
      break;
  }
}


// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void HFIBLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {

  float center;
  int sector;
  float _ca,_sa;

  switch (foc_modulation)
  {
    case FOCModulationType::Trapezoid_120 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
      // determine the sector
      sector = 6 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_120_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_120_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON);// disable phase if possible
      }else{
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF);// disable phase if possible
      }

    break;

    case FOCModulationType::Trapezoid_150 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
      // determine the sector
      sector = 12 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_150_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][2]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF); // disable phase if possible
      }else{
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // enable all phases
      }

    break;

    case FOCModulationType::SinePWM :
    case FOCModulationType::SpaceVectorPWM :
      // Sinusoidal PWM modulation
      // Inverse Park + Clarke transformation
      _sincos(angle_el, &_sa, &_ca);

      // Inverse park transform
      Ualpha =  _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
      Ubeta =  _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;

      // Clarke transform
      Ua = Ualpha;
      Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
      Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

      center = driver->voltage_limit/2;
      if (foc_modulation == FOCModulationType::SpaceVectorPWM){
        // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
        // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
        // Midpoint Clamp
        float Umin = min(Ua, min(Ub, Uc));
        float Umax = max(Ua, max(Ub, Uc));
        center -= (Umax+Umin) / 2;
      } 

      if (!modulation_centered) {
        float Umin = min(Ua, min(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
      }else{
        Ua += center;
        Ub += center;
        Uc += center;
      }

      break;

  }
  
  // set the voltages in driver
  driver->setPwm(Ua, Ub, Uc);
}



// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float HFIBLDCMotor::velocityOpenloop(float target_velocity){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
  // for display purposes
  shaft_velocity = target_velocity;

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)){
    Uq = _constrain(current_limit*phase_resistance + fabs(voltage_bemf),-voltage_limit, voltage_limit);
    // recalculate the current  
    current.q = (Uq - fabs(voltage_bemf))/phase_resistance;
  }
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float HFIBLDCMotor::angleOpenloop(float target_angle){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
  //                        where small position changes are no longer captured by the precision of floats
  //                        when the total position is large.
  if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts)){
    shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts;
    shaft_velocity = velocity_limit;
  }else{
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)){
    Uq = _constrain(current_limit*phase_resistance + fabs(voltage_bemf),-voltage_limit, voltage_limit);
    // recalculate the current  
    current.q = (Uq - fabs(voltage_bemf))/phase_resistance;
  }
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  // sensor precision: this calculation is OK due to the normalisation
  setPhaseVoltage(Uq,  0, _electricalAngle(_normalizeAngle(shaft_angle), pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}
