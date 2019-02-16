package frc.robot.commands.arm;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.PositionEnum;

public class MoveToHeightCommand extends CommandGroup {
    private double height;
    private final double armInitialLength = 18; //Length of the arm from pivot point without extension in inches
    private final double pivotHeight = 29.75; //Height of point of rotation for the arm in inches

    public MoveToHeightCommand(PositionEnum position, double groundProjection){
        height = position.getValue();

        double calculationHeight = height - pivotHeight;

        addParallel(new ExtendArmToPositionCommand(
            armInitialLength - Math.sqrt(calculationHeight * calculationHeight + groundProjection * groundProjection)));
        addParallel(new RotateArmToAngleCommand(Math.toDegrees(Math.atan(calculationHeight / groundProjection))));
    }
}