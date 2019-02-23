package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.drive.shift.AutomaticGearShiftCommand;

public class GearShifterSubsystem extends Subsystem {


    //physical devices
    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(RobotMap.GEARSHIFT_PCM_ID,
            RobotMap.GEARSHIFTUP_SOLENOID_PCM, RobotMap.GEARSHIFTDOWN_SOLENOID_PCM);

    public enum Gear {        
        HIGH_GEAR (DoubleSolenoid.Value.kReverse),      //### need to check right order
        LOW_GEAR (DoubleSolenoid.Value.kForward) ;
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