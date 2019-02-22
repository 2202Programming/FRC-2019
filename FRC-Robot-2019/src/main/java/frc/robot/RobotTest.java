package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.arm.tests.TestArmRateCmd;
import frc.robot.commands.intake.tests.TestWristRateCommand;
import frc.robot.commands.intake.tests.TestWristPositionCommand;

public class RobotTest {

    // TESTING Started in TestInit
    Command testWristCmd;
    TestArmRateCmd testArmCmd;

    public RobotTest() {
        // TESTING Commands, only get scheduled if we enter Test mode
        testWristCmd = new  TestWristRateCommand();
                            //TestWristPositionCommand();

       // testArmCmd = new TestArmRateCmd();

    }

    public void initialize() {
        // testArmCmd.start();
        testWristCmd.start();
    }

    public void periodic() {

    }

    /**
     * Bind any testing buttons/controls here
     */
    public void OI() {

    }

}