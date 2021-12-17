package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AutomaticUpShiftCommand extends ParallelCommandGroup {
    public AutomaticUpShiftCommand() {
        this.addCommands( 
            new UpShiftCommand(),
            new ThrottleCommand(1.0, 0.5, 1)
            );
    }
}