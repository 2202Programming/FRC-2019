package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.RobotMap;

public class GearShifterSubsystem extends Subsystem {
    private Solenoid gearShiftSolenoid = new Solenoid(RobotMap.GEARSHIFT_SOLENOID_CAN_ID);

    public GearShifterSubsystem() {
        addChild("PCM", (Sendable) gearShiftSolenoid);
    }

    public void initDefaultCommand() {
        
    }

}