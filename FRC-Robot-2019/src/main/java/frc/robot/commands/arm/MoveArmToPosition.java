/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.util.MathUtil;
import frc.robot.commands.util.RateLimiter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Position;

public class MoveArmToPosition extends Command {
    double timeout;
    double height;
    double projection;
    double error;
    private ArmSubsystem arm;

    // Phyical values from sub-systems as needed
    private Position armPosition;
    private ArmStatePositioner armPositioner;
    private RateLimiter heightLimiter;
    private RateLimiter projectionLimiter;

    public MoveArmToPosition(double height, double projection, double error, double timeout) {
        this.height = height;
        this.projection = projection;
        this.timeout = timeout;
        this.error = Math.abs(error);

        arm = Robot.arm;
        armPositioner = Robot.arm.getArmPositioner();
        projectionLimiter = armPositioner.getProjectionLimiter();
        heightLimiter = armPositioner.getHeightLimiter();
    }

    @Override
    protected void initialize() {
        setTimeout(timeout);
        heightLimiter.initialize();
        projectionLimiter.initialize();
        execute();
    }

    @Override
    protected void execute() {
        heightLimiter.execute();
        projectionLimiter.execute();
        double h_cmd = heightLimiter.get();
        double x_cmd = projectionLimiter.get();
        double heightAbovePivot = h_cmd - arm.ARM_PIVOT_HEIGHT;

        // Adjusts x_cmd so h_cmd is always reached
        if (heightAbovePivot * heightAbovePivot + x_cmd * x_cmd >= arm.MAX_ARM_LENGTH * arm.MAX_ARM_LENGTH) {
            // Assumes h_cmd doesn't go above the max attainable height
            x_cmd = Math.sqrt(arm.MAX_ARM_LENGTH * arm.MAX_ARM_LENGTH - heightAbovePivot * heightAbovePivot);
        }

        // Calculate the angle
        double calculatedAngle = 0.0;
        if (Math.abs(x_cmd) >= 1e-6) {
            calculatedAngle = 90 - Math.toDegrees(Math.atan(heightAbovePivot / x_cmd));
        }
        double curAngle = MathUtil.limit(calculatedAngle, arm.PHI_MIN, arm.PHI_MAX);

        // Calculate extension based on current angle
        double calculatedExtension = heightAbovePivot;
        if (Math.abs(arm.getRealAngle()) >= 1e-6) {
            calculatedExtension = (x_cmd / Math.sin(arm.getRealAngle())) - arm.ARM_BASE_LENGTH - arm.WRIST_LENGTH;
        }
        // Limiting here is technically unnecessary because limiting is also done in
        // setExtension
        double extensionLength = MathUtil.limit(calculatedExtension, arm.EXTEND_MIN, arm.EXTEND_MAX);

        arm.setAngle(curAngle);
        arm.setExtension(extensionLength);
    }
    
    @Override
    protected boolean isFinished() {
        armPosition = Robot.arm.getArmPosition();
        double h_err = Math.abs(armPosition.height - height);
        double x_err = Math.abs(armPosition.projection - projection);
        boolean posGood = (h_err < error) && (x_err < error);
        return posGood || isTimedOut();
    }
}