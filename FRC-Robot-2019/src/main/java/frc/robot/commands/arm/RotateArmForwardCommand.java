<<<<<<< Updated upstream:FRC-Robot-2019/src/main/java/frc/robot/commands/RotateArmForwardCommand.java
package frc.robot.commands;

=======
package frc.robot.commands.arm;
>>>>>>> Stashed changes:FRC-Robot-2019/src/main/java/frc/robot/commands/arm/RotateArmForwardCommand.java
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;

public class RotateArmForwardCommand extends Command {
    public RotateArmForwardCommand() {
        requires(Robot.arm);
    }

    @Override
    protected void initialize() {
        Robot.arm.stopRotation();
    }

    @Override
    protected void execute() {
        Robot.arm.rotateForward();
    }

    @Override
    protected void end() {
        Robot.arm.stopRotation();
    }

    @Override
    protected boolean isFinished() {
        return Robot.m_oi.getController1().getRawButtonReleased(XboxControllerButtonCode.X.getCode());
    }
}