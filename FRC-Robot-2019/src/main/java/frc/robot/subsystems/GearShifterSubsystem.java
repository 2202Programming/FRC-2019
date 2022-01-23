package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Robot;
import static frc.robot.RobotMap.*;
import frc.robot.commands.drive.shift.AutomaticGearShiftCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearShifterSubsystem extends SubsystemBase {
    private long logTimer;
    private boolean isAutoShiftEnabled;

    public GearShifterSubsystem() {
        logTimer = System.currentTimeMillis();
    }

    public void autoshiftEnabled(boolean temp) {
        isAutoShiftEnabled = temp;
    }

    public void log(int interval) {

        if ((logTimer + interval) < System.currentTimeMillis()) { //only post to smartdashboard every interval ms
          logTimer = System.currentTimeMillis();
          SmartDashboard.putBoolean("Autoshift Enabled", isAutoShiftEnabled);  
          SmartDashboard.putString("Gear Shifter State", String.valueOf(getCurGear()));
        }
      }

    //physical devices
    private DoubleSolenoid gearShiftSolenoid = new 
        DoubleSolenoid(GEARSHIFT_PCM_ID, PneumaticsModuleType.CTREPCM,
        GEARSHIFTUP_SOLENOID_PCM, GEARSHIFTDOWN_SOLENOID_PCM);

    public enum Gear {        
        HIGH_GEAR (kForward),  
        LOW_GEAR (kReverse) ;
        private final DoubleSolenoid.Value gearCode;
        Gear(DoubleSolenoid.Value value) { gearCode = value; }
        public DoubleSolenoid.Value solenoidCmd() {return this.gearCode; }
    }

    //State
    Gear curGear = Gear.LOW_GEAR;    // really can grab this from the 
    double shiftPoint;

    public GearShifterSubsystem(double _shiftPoint) {
        addChild("GearShifter", (Sendable) gearShiftSolenoid);
        this.shiftPoint = _shiftPoint;
    }


    public void initDefaultCommand() {
        shiftDown();
        setDefaultCommand(new AutomaticGearShiftCommand());
        isAutoShiftEnabled = true;
    }

    public void shiftUp()  {
        // dpl - I don't think we need the conditional checks to see if we are in gear. just do it.
        gearShiftSolenoid.set(Gear.HIGH_GEAR.solenoidCmd());
        curGear = Gear.HIGH_GEAR;
    }

    public void shiftDown() {
        double speed = (Robot.driveTrain.getLeftEncoderTalon().getSelectedSensorVelocity()
                + Robot.driveTrain.getRightEncoderTalon().getSelectedSensorVelocity()) / 2;
        if ( speed <= shiftPoint) 
        {
            gearShiftSolenoid.set(Gear.LOW_GEAR.solenoidCmd());
            curGear = Gear.LOW_GEAR;
        }
    }

    public Gear getCurGear() {
        return curGear;
    }
}