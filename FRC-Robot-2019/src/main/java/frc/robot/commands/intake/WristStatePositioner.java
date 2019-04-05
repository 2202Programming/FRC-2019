package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.CommandManager.Modes;
import frc.robot.commands.util.Angle;

public class WristStatePositioner extends Command {
    private double curAngle;
    private Modes prevMode;

    // 3D tensor of angles: format is [state][inversionStatus][index]
    private double[][][] angles = { { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Construction
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Setting Zeros
            { { Angle.Starting_Hatch_Hunt.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // HuntGameStart
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // HuntingHatch
            { { Angle.Perpendicular_Down.getAngle() }, { Angle.Perpendicular_Down.getAngle() } }, // HuntingCargo
            { { Angle.Perpendicular_Down.getAngle() }, { Angle.Perpendicular_Down.getAngle() } }, // HuntingFloor
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Capturing
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Recapturing
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Drive
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Defense
            { { Angle.Hatch_Delivery.getAngle() }, { Angle.Back_Hatch_Delivery.getAngle() } }, // DeliverHatch
            { { Angle.Cargo_Delivery.getAngle() }, { Angle.Back_Cargo_Delivery.getAngle() } }, // DeliverCargo
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Flipping
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Releasing
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Climbing
    };

    public WristStatePositioner() {
        requires(Robot.intake);
    }

    @Override
    protected void execute() {
        // Update position based on current mode
        Modes curMode = Robot.m_cmdMgr.getCurMode();
        int invert = Robot.arm.isInverted() ? 1 : 0;
        int index = Robot.m_cmdMgr.getPositionIndex();
        if (curMode != prevMode) {
            // Update position only if state changes to allow something to override position
            // for that state
            curAngle = angles[curMode.get()][invert][index];
            prevMode = curMode;
        }

        // intake angle is relative to arm
        double offset = Robot.arm.getRealAngle() - curAngle;
        Robot.intake.setAngle(offset);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}