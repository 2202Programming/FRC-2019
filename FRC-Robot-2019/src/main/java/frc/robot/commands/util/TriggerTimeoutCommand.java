package frc.robot.commands.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TriggerTimeoutCommand extends CommandBase {
    private BooleanSupplier event; 
    private double timeout;

    public TriggerTimeoutCommand(BooleanSupplier func, Double timeout) {
        event = func;
        this.timeout = timeout;
    }

    @Override
   public void initialize() {
        setTimeout(timeout);
    }

    @Override
    public boolean isFinished() {
        return isTimedOut() || event.getAsBoolean();
    }
}
