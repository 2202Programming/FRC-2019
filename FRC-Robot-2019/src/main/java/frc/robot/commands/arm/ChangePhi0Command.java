package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ChangePhi0Command extends Command {
    public static final double EPSILON = 1E12;

    private double incrementValue;
    private double initialPhi0 = Robot.arm.getPhi0();

    /**
     * Creates a ChangePhi0Command.
     * Changes <code>PHI0</code> of the Arm Subsystem up or down 5 degrees, depending directly on whether or not <code>positive</code> is true.
     * @param positive whether or not the incrementValue for <code>PHI0</code> should be positive.
     */
    public ChangePhi0Command(boolean positive) {
        requires(Robot.arm);
        incrementValue = 5;
        if (!positive) incrementValue *= -1;
    }

    /**
     * Creates a ChangePhi0Command.
     * Changes <code>PHI0</code> of the Arm Subsystem by a given value.
     * @param phi0IncrementValue the value about which <code>PHI0</code> of the ArmSubsystem should increment.
     */
    public ChangePhi0Command(double phi0IncrementValue) {
        requires(Robot.arm);
        incrementValue = phi0IncrementValue;
    }

    @Override
    protected void initialize() {
        Robot.arm.setPhi0(initialPhi0 + incrementValue);
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(Robot.arm.getPhi0() - initialPhi0 - incrementValue) < EPSILON;
    }

}