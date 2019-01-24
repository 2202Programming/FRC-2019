package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.RobotMap;
import frc.robot.commands.AutomaticGearShiftCommand;

public class GearShifterSubsystem extends Subsystem {
    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(RobotMap.GEARSHIFT_SOLENOID_CAN_ID, RobotMap.GEARSHIFTUP_SOLENOID_ID, RobotMap.GEARSHIFTDOWN_SOLENOID_ID);

    public static enum Gear {
        LOW_GEAR, HIGH_GEAR
    }

    private Gear curGear;
    
    public GearShifterSubsystem() {
        addChild("PCM", (Sendable) gearShiftSolenoid);
    }

    public void initDefaultCommand() {
        curGear = Gear.HIGH_GEAR;
        shiftDown();
        setDefaultCommand(new AutomaticGearShiftCommand());
    }

    public void shiftUp() {
        if(curGear != Gear.HIGH_GEAR)
            gearShiftSolenoid.set(DoubleSolenoid.Value.kForward);
        curGear = Gear.HIGH_GEAR;
    }

    public void shiftDown() {
        if(curGear != Gear.LOW_GEAR)
            gearShiftSolenoid.set(DoubleSolenoid.Value.kReverse);
        curGear = Gear.LOW_GEAR;
    }

    public Gear getCurGear() {
        return curGear;
    }
}