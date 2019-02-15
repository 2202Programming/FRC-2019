package frc.robot.commands.arm;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.PositionEnum;

public class MoveToHeightCommand extends CommandGroup {
    private double height;
    private final double armInitialLength = 10;
    private final double pivotHeight = 42;

    public MoveToHeightCommand(PositionEnum position, double groundProjection){
        height = position.getValue();

        double calculationHeight = height - pivotHeight;

        addParallel(new ExtendArmToPositionCommand(
            armInitialLength - Math.sqrt(calculationHeight * calculationHeight + groundProjection * groundProjection)));
        addParallel(new RotateArmToAngleCommand(Math.toDegrees(Math.atan(calculationHeight / groundProjection))));
    }
}