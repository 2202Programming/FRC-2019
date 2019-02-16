package frc.robot.commands.arm;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveArmAtHeight extends Command {
    //Length of the arm from pivot point without extension in inches
    private final double armInitialLength = 18;
    //Height of point of rotation for the arm in inches
    private final double pivotHeight = 29.75;
    //TODO: Legit length
    //Starting projection of arm (starts at edge of bumper) in inches
    private final double projectionInitialLength = 16;

    //Make an h' to more easily construct a triangle
    private double calculationHeight;

    public MoveArmAtHeight(double height){
        requires(Robot.arm);
        calculationHeight = height - pivotHeight; 
    }

    protected boolean isFinished() {
        return false;
    }
}