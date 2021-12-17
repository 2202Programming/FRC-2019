package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class WristSetAngleCommand extends InstantCommand {
    private double angle;

    /**
     * Sets the wrist to a specific angle
     * */
    public WristSetAngleCommand(double angle) {   
        addRequirements(Robot.intake);
        this.angle = angle;
    }

    @Override
   public void initialize() {
        Robot.intake.setAngle(angle);   
    }
}