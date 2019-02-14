package frc.robot.commands.arm;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;
/**
 * Rotates the arm backwards.
 */
public class RotateArmBackwardCommand extends Command {
   /**
     * Constructor for the rotate arm backwards command. Requires an arm subsystem.
     */
    public RotateArmBackwardCommand() {
        requires(Robot.arm);
    }
    /**
     * Initializes, and stops the rotation of the arm.
     */
    @Override
    protected void initialize() {
        Robot.arm.stopRotation();
        
    }
    /**
     * Rotates the arm backwards.
     */
    @Override
    protected void execute() {
        Robot.arm.rotateBackward();
    }
    /**
     * Ends, and stops the rotation of the arm.
     */
    @Override
    protected void end() {
        Robot.arm.stopRotation();
    }
    /**
     * @return True if the B button (rotator) is released.
     * @return False if the B button (rotator) is still held.
     */
    @Override
    protected boolean isFinished() {
        return Robot.m_oi.getController1().getRawButtonReleased(XboxControllerButtonCode.B.getCode());
    }
}