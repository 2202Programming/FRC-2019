package frc.robot.commands.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.command.Command;

public class TriggerTimeoutCommand extends Command {
    private BooleanSupplier event; 
    private double timeout;

    public TriggerTimeoutCommand(BooleanSupplier func, Double timeout) {
        event = func;
        this.timeout = timeout;
    }

    @Override
    protected void initialize() {
        setTimeout(timeout);
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut() || event.getAsBoolean();
    }
}
