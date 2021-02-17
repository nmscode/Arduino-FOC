#include "pid.h"

PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // output derivative limit [volts/second]
    , limit(limit)         // output supply limit     [volts]
    , integral_prev(0.0)
    , error_prev(0.0)
    , output_prev(0.0)
{
    timestamp_prev = _micros();
}

// PID controller function
float PIDController::operator() (float error){
    // calculate the time from the last call
    unsigned long timestamp_now = _micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part 
    // u_p  = P *e(k)
    float proportional = P * error;
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = integral_prev + I*Ts*0.5*(error + error_prev);
    // antiwindup - limit the output
    integral = _constrain(integral, -limit, limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative = D*(error - error_prev)/Ts;

    // sum all the components
    float output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    output = _constrain(output, -limit, limit);

    // limit the acceleration by ramping the output
    float output_rate = (output - output_prev)/Ts;
    if (output_rate > output_ramp)
        output = output_prev + output_ramp*Ts;
    else if (output_rate < -output_ramp)
        output = output_prev - output_ramp*Ts;
        
    // saving for the next pass
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}


String PIDController::communicate(String user_cmd){
  String ret = "";
  char cmd = user_cmd.charAt(0);
  char GET  = user_cmd.charAt(1) == '\n';
  float value = user_cmd.substring(1).toFloat();

  switch (cmd){
    case 'P':      // P gain change
      ret = ret + "P: ";
      if(!GET) P = value;
      ret = ret + P;
      break;
    case 'I':      // I gain change
      ret = ret + "I: ";
      if(!GET) I = value;
      ret = ret + I;
      break;
    case 'D':      // D gain change
      ret = ret + "D: ";
      if(!GET) D = value;
      ret = ret + D;
      break;
    case 'R':      //  ramp change
      ret = ret + "ramp: ";
      if(!GET) output_ramp = value;
      ret = ret + output_ramp;
      break;
    case 'L':      //  limit change
      ret = ret + "limit: ";
      if(!GET) limit = value;
      ret = ret + limit;
      break;
    default:
      ret = ret + F("error");
      break;
  }
  return ret; // not well handled
}