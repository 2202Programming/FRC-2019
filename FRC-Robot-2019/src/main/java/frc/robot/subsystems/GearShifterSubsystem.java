import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class GearShifterSubsystem extends Subsystem {
    private Solenoid gearShiftSolenoid = new Solenoid(RobotMap.GEARSHIFT_SOLENOID_CAN_ID);

    public GearShifterSubsystem() {

    }

    public void initDefaultCommand() {
        
    }

}