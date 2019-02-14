package frc.robot.commands.util;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.lang.Math;
//import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;


public class RateCommand  {
    double dT = Robot.dT;  //sample period seconds - use robots

    // expo only works on normalize inputs
    double kExpo;          // 0.0 to 1.0 for flatness 0 -> straight curve
    double kCexpo;         // complement to expo 

    // dx in device units of setter/getter functions
    double dx_min;         // min dx/dt (device units)/second
    double dx_max;         // max dx/dt (device units)/second
  
    // dead zone is in normalized input units -1.0 to 1.0
    // The zone doesn't have to be symetric.
    // min is the left hand side, typically -1 to 0 range
    // max is right hand side, typically 0 to +1 range
    final double kInputMag = 1.0;   //assume symetric -1 to 1
    double dz_min;         // dead zone min
    double dz_max;         // dead zone max
    double dz_center;      // dead zone center - might not be zero
    double dz_min_scale;   // correct full scale gain for deadzone
    double dz_max_scale;   // correction for full scale

    DoubleSupplier cmdFunct; 
    DoubleConsumer devSetter;   // function to read subsystem being controlled
    DoubleSupplier devGetter;   // function to write subsystem being controlled

    double x_max; // max position of device in device units
    double x_min; // min position of device in device units
    double X;   //dev units

    public RateCommand(
          //Subsystem system,
          DoubleSupplier _cmdFunct, 
          DoubleSupplier getter, 
          DoubleConsumer setter,
                 double dev_min,
                 double dev_max,
                 double _dx_min,
                 double _dx_max,
                 double _dz_min,
                 double _dz_max,
                 double expo ) {
        //requires(system);
        setExpo(expo);
        setDeadZone(_dz_min, _dz_max);
        devGetter = getter;
        devSetter = setter;
        cmdFunct = _cmdFunct;
        x_max = dev_max;
        x_min = dev_min;
        dx_min = _dx_min;
        dx_max = _dx_max;
        setDeadZone(_dz_min, _dz_max);        
        X = 0.0;
    }

    public double getCmd() {
      if (cmdFunct != null) return cmdFunct.getAsDouble();
      return 0.0;
    };

    public void initialize() {
      X = devGetter.getAsDouble();   // start where we are according to the device
    }

    public void execute() {
    //Shape the input command
    double cmd = deadZone(getCmd());
    cmd = (kExpo != 0.0) ?  expo(cmd) : cmd;
    
    // scale the command
    double dX = (cmd > dz_center) ? cmd *dx_max  : cmd * dx_min; 
 
    // integrate the position with the 
    X = X + dX * dT;
    limitX();    // dont command more than we can take, prevent wind up

    // outout the new postionion command
    this.devSetter.accept(X);
  }

  

  public double setExpo(double a) {
      if (a > 1.0) a = 1.0;
      if (a < 0.0) a = 0.0;
      kExpo = a;
      kCexpo = 1.0 - a;
      return kExpo;
  }

  // set dz values and compute correcting scales so we get max deflections 
  public void setDeadZone(double min, double max) {
    dz_center = (max - min) / 2.0;
    dz_max = max;
    dz_min = min;  

    // fix scaling of input so full scale is seen even with dead zone
    dz_max_scale = Math.abs(kInputMag / (kInputMag - dz_max));
    dz_min_scale = Math.abs(kInputMag / (-kInputMag - dz_min));
  }

// helper functions
  // Make sure our position X never execeeds its limits
  private void limitX() {
    if (X > x_max) X = x_max;
    if (X < x_min) X = x_min;
  }

  // apply deadzone to input in and scale to get propper range
  private double deadZone(double x) {
    // in the dead zone, ignore it
    if ((x > dz_min) && (x < dz_max) ) return (double)0.0;
    // handle positive x command
    if (x > dz_center) {
      return (x - dz_max)*dz_max_scale;
    }
    // left of dz_center
    return (x - dz_min)*dz_min_scale;
  }

  // use Gord W's expo function
  private double expo(double x) {
    return x*x*x*kExpo + kCexpo*x;
  }
}
