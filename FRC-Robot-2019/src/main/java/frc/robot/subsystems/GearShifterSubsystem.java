package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.RobotMap;

public class GearShifterSubsystem extends Subsystem {
    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(RobotMap.GEARSHIFT_SOLENOID_CAN_ID, RobotMap.GEARSHIFTUP_SOLENOID_ID, RobotMap.GEARSHIFTDOWN_SOLENOID_ID);

    public GearShifterSubsystem() {
        addChild("PCM", (Sendable) gearShiftSolenoid);
    }

    public void initDefaultCommand() {
        
    }

    public void shiftUp() {
        gearShiftSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void shiftDown() {
        gearShiftSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

}