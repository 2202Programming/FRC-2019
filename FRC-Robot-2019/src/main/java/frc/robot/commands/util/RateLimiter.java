package frc.robot.commands.util;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/** 
 *  limit output position based on desired physical limits
 *       postion min/max
 *        rate limited
 *  Deals with input being a Postion or a Rate input
 *
 *  Similar to https://www.mathworks.com/help/simulink/slref/ratelimiter.html
 *
*/
public class RateLimiter {
  double dT; // sample period seconds - use robots in most cases

  public enum InputModel {
    Rate, // input cmd is a rate, need integration to get X
    Position // input is a position, just shape X output.
  }

  InputModel model;

  // dx in device units of setter/getter functions
  double dx_fall; // falling dx/dt (device units)/second
  double dx_raise; // raising dx/dt (device units)/second

  
  // dead zone is in physical units, after kRate gain is applied
  // max is right hand side
  double dz_mag;   // dead zone min
  double kRate = 1.0;   // K rate gain, example applied to joystick
  double dz_min_scale; // correct full scale gain for deadzone
  double dz_max_scale; // correction for full scale

  // device limits
  double x_max; // max command value for device (device units)
  double x_min; // min command value for device (device units)
  final DoubleSupplier inFunct;   // function input X
  final DoubleConsumer devSetter; // function to write device being controlled
  final DoubleSupplier devGetter; // function to read device being controlled

  // input vars - read on execute() and init()
  double cmd;
  double devPos;
  double devPrev;
  
  // output vars
  double X;      // output value (device units)
  double Xprev;  // previous frame output

  public RateLimiter(final Double dT,
      final DoubleSupplier inFunct, 
      final DoubleSupplier getter, // optional
      final DoubleConsumer setter, // optional
      double x_min, 
      double x_max, 
      double dx_fall,  
      double dx_raise,
      final InputModel model) 
  {
    this.dT = dT;
    devGetter = getter;
    devSetter = setter;
    this.inFunct = inFunct;
    this.x_max = x_max;
    this.x_min = x_min;
    this.dx_fall = dx_fall;
    this.dx_raise = dx_raise;

    setRateGain(1.0);
    setDeadZone(0.0);            
    this.model = model;
    initialize();
  }

  // Output of RateLimiter, setter is optional, so also have this availalble
  public double get() {
    return X;
  }

  public void setRateGain(double k)
  {
    kRate = k;
  }


  // designed to be called from an FRC Command if needed
  public void initialize() {
    getInputs();   // sets cmd, devPos member vars

    // start where we are according to the device if we have one
    // otherwise use the initial value of the input command, cmd.
    Xprev =  (devPos != Double.NaN)  ? devPos: cmd;
    devPrev = devPos;
  }

  // designed to be called from FRC Command if needed, call once per frame
  // and no more because it does an integration in Rate mode.
  public void execute() {
    getInputs();          // reads input command and device
    double dX = dX();     // computs a dX (rate limited, deadzoned)
    double x = Xprev + dX * dT;  // integrate the dX desired rate limited X
    
    // update output vars
    X =limit(x, x_min, x_max);  // apply end limits
    Xprev = X;
    putOutput();                // output to the device if required 
  }
  
  // set dz values and compute correcting scales so we get max deflections
  public void setDeadZone(double dz_mag) {  
    this.dz_mag = Math.abs(dz_mag);
    
    // fix scaling of input so full scale is seen even with dead zone
    //dz_max_scale = Math.abs(kInputMag / (kInputMag - dz_max));
    //dz_min_scale = Math.abs(kInputMag / (-kInputMag - dz_min));
  }

  // helper functions
  private double dX(){   
    double dX=0.0;
    double kc = cmd * kRate;   //input cmd * K gain (units or units/sec)

    if (model == InputModel.Rate) {
      // In rate mode we treat input as a dX to integrate to get desired X
      // This way we can limit X's growth rate and position.
      //rates need dz on inputs, typically joysticks
      double cmdDz = deadZone(kc);

      dX = limit(cmdDz, dx_fall, dx_raise);
    }
    if (model == InputModel.Position) {
      // for postion limits, need to look at command vs current an limit rate aka dX
      double rate = (kc - Xprev)/dT;   
      double phyRate = (devPos - devPrev) /dT;  //this could amplify noise
      dX = limit(rate, dx_fall, dx_raise);
    }
    return dX;
  }

  // Make sure our position X never execeeds its limits
  private double limit(double x, double min, double max) {
    if (x > max)  x = max;
    if (x < min)  x = min;
    return x;
  }

  // apply deadzone to input in and scale to get propper range
  private double deadZone(double x) {
    if (Math.abs(x) < dz_mag)  return (double) 0.0;
    return x;
  }

  // Read input functions for cmd and device if we have them
  private void getInputs() {
    devPos = (devGetter != null)  ? devGetter.getAsDouble() : Double.NaN;
    cmd = (inFunct != null)       ? inFunct.getAsDouble()   : 0.0;
  };

  // send the restult if we can
  private void putOutput() {
    if (devSetter != null)
      devSetter.accept(X);
  }
}
