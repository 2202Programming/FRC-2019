package frc.robot.commands.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TriggerTimeoutCommand extends WaitCommand {
    private BooleanSupplier event; 
    
    public TriggerTimeoutCommand(BooleanSupplier func, Double timeout) {
        super(timeout);
        event = func;
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || event.getAsBoolean();
    }
}
