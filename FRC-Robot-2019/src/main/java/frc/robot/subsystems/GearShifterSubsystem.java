package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.RobotMap;

public class GearShifterSubsystem extends Subsystem {
    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(RobotMap.GEARSHIFT_SOLENOID_CAN_ID, RobotMap.GEARSHIFTUP_SOLENOID_ID, RobotMap.GEARSHIFTDOWN_SOLENOID_ID);

    private enum Gear {
        LOW_GEAR, HIGH_GEAR
    }

    private Gear currentGear = Gear.LOW_GEAR;
    
    public GearShifterSubsystem() {
        addChild("PCM", (Sendable) gearShiftSolenoid);
    }

    public void initDefaultCommand() {
        
    }

    public void shiftUp() {
        if(currentGear == Gear.LOW_GEAR)
            gearShiftSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void shiftDown() {
        if(currentGear == Gear.HIGH_GEAR)
            gearShiftSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

}