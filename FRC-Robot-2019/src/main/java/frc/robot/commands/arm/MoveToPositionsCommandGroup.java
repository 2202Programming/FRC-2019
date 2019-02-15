package frc.robot.commands.arm;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.PositionEnum;
import frc.robot.commands.intake.RotateCommandGroup;
public class MoveToPositionsCommandGroup extends CommandGroup {
    private PositionEnum positions;
    private double positionValue;
    private final double XPLACEHOLDER = 30.0;
    public MoveToPositionsCommandGroup(PositionEnum positions){
        this.positions = positions;
        positionValue = positions.getValue();
        addParallel(new RotateCommandGroup(XPLACEHOLDER, positions.getValue()));
    }
}