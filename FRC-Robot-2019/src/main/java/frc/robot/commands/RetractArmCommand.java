package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;
/**
 * Retracts the arm on the robot chassis.
 */
public class RetractArmCommand extends Command {
    /**
     * Constructor for the retract arm command. Requires an arm subsystem.
     */
    public RetractArmCommand() {
        requires(Robot.arm);
    }
    /**
     * Initializes, and stops the extension of the arm.
     */
    @Override
    protected void initialize() {
        Robot.arm.stopExtension();
    }
    /**
     * Retracts the arm.
     */
    @Override
    protected void execute() {
        Robot.arm.retract();
    }
    /**
     * Ends, and stops the extension of the arm.
     */
    @Override
    protected void end() {
        Robot.arm.stopExtension();
    }
    /**
     * @return true if the A button (extender) is released.
     * @return false if the A button (extender) is still held.
     */
    @Override
    protected boolean isFinished() {
        return Robot.m_oi.getController1().getRawButtonReleased(XboxControllerButtonCode.A.getCode());
    }
}