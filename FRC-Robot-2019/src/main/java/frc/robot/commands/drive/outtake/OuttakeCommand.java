package frc.robot.commands.drive.outtake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.input.XboxControllerButtonCode;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends Command {
    private IntakeSubsystem intake;

    public OuttakeCommand() {
        requires(Robot.intake);
        intake = Robot.intake;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        intake.vacuumOff();
    }

    @Override
    protected void end() {
        intake.vacuumOff(); // shouldn't be needed.
    }

    @Override
    protected boolean isFinished() {
        /**
         * We once the motor is turned off, this command is finished, you shouldn't have
         * to wait for a button release
         * Robot.m_oi.getAssistentController().getRawButtonReleased(XboxControllerButtonCode.RB.getCode());
         *
         */
        return true;
    }

    @Override
    protected void interrupted() {
        return;
    }

}
