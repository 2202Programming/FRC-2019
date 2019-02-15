package frc.robot.commands.arm;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.PositionEnum;

public class MoveToHeightCommand extends CommandGroup {
    private double height;
    private final double groundProjection = 30.0;
    private final double armInitialLength = 10;
    private final double pivotHeight = 42; //TODO: Factor in height of pivot point

    public MoveToHeightCommand(PositionEnum position){
        height = position.getValue();

        addParallel(new ExtendArmToPositionCommand(
            armInitialLength - Math.sqrt(height * height + groundProjection * groundProjection)));
        addParallel(new RotateArmToAngleCommand(Math.toDegrees(Math.atan(height / groundProjection))));
    }
}