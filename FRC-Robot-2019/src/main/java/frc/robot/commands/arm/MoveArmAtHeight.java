package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveArmAtHeight extends Command {
    // Length of the arm from pivot point without extension in inches
    private final double armInitialLength = Robot.arm.EXTEND_MIN + Robot.arm.ARM_BASE_LENGTH
            + Robot.intake.WristDistToPivot;
    // Height of point of rotation for the arm in inches
    private final double pivotHeight = Robot.arm.ARM_PIVOT_HEIGHT;

    /*
     * TODO: Legit length Starting projection of arm (starts at edge of bumper) in
     * inches
     */
    private final double xCenter = (Robot.kProjectConstraint + Robot.kForwardProjectMin) / 2;
    private final double projectionInitialLength = xCenter; // TODO:FIX this to take account for which side arm is on
    // there will be two limit - forward side and rear side. - Derek/Shawn

    DoubleSupplier getHeight;

    // Maximum projection based on
    private final double projectionMax = Robot.kForwardProjectMin + Robot.kProjectConstraint;

    // Make an h' to more easily construct a triangle
    private double calculationHeight;

    // Projection of the arm on the ground
    private double xProjection;

    DoubleSupplier heightFunct;

    public MoveArmAtHeight(DoubleSupplier heightFunct) {
        requires(Robot.arm);
        this.heightFunct = heightFunct;
    }

    protected void execute() {
        double height = heightFunct.getAsDouble();

        // If target is below pivot height
        boolean belowPiv = height < pivotHeight;
        if (!belowPiv)
            calculationHeight = height - pivotHeight;
        else
            calculationHeight = pivotHeight - height;
        xProjection = projectionInitialLength;

        double x_cmdRaw = Robot.m_oi.getAssistantController().getY();
        // TODO: Mapping joystick properly to change in projection
        xProjection += x_cmdRaw * 0.3; // Raw to inches integration

        if (xProjection > projectionMax)
            xProjection = projectionMax;

        // Rotate to maintain height as projection changes
        double angle = Math.toDegrees(Math.atan(calculationHeight / xProjection));
        angle += (belowPiv) ? 90.0 : 0.0;

        double extension = Math.sqrt(calculationHeight * calculationHeight + xProjection * xProjection);
        extension -= armInitialLength;
        // Extend to allow for change in projection
        Robot.arm.setExtension(extension);
        Robot.arm.setAngle(angle);
        /*
         * Alternative extension calculation xProjection /
         * Math.cos(Robot.arm.getAngle()) - armInitialLength
         */
    }

    protected boolean isFinished() {
        return false;
    }
}