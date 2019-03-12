package frc.robot.commands.intake; 

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WristTrackAngle extends Command{
    private double angle;

    /**
     * Makes the wrist track a specific angle from vertical
     */
    public WristTrackAngle(double trackedAngle){
        requires(Robot.intake);
        angle = trackedAngle;
    }

    @Override
    protected void execute() {
        //intake angle is relative to arm
        double offset = angle - Robot.arm.getRealAngle();
        Robot.intake.setAngle(offset) ;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    public enum Angle {
        Perpendicular_Up(0.0), 
        Cargo_Delivery(60.0),
        Starting_Hatch_Hunt(76.0),
        Hatch_Delivery(83.0),
        Parallel(90.0),
        Perpendicular_Down(180.0);

        private double phi;

        Angle(double phi) {
            this.phi = phi;
        }

        public double getAngle() {
            return phi;
        }
    }
}