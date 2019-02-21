package frc.robot;

import frc.robot.commands.arm.tests.TestArmRateCmd;
import frc.robot.commands.intake.tests.TestWristRateCommand;

public class RobotTest {

    // TESTING Started in TestInit
    TestWristRateCommand testWristCmd;
    TestArmRateCmd testArmCmd;

    public RobotTest() {
        // TESTING Commands, only get scheduled if we enter Test mode
        testWristCmd = new TestWristRateCommand();
        testArmCmd = new TestArmRateCmd();

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