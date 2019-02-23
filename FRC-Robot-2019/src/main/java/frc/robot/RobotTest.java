package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.input.XboxControllerButtonCode;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.arm.tests.TestArmRateCmd;
import frc.robot.commands.intake.tests.TestWristRateCommand;
import frc.robot.commands.intake.tests.TestWristPositionCommand;
import frc.robot.commands.intake.VacuumCommand;
import frc.robot.commands.drive.shift.DownShiftCommand;
import frc.robot.commands.drive.shift.UpShiftCommand;


public class RobotTest {
    private XboxController driver = new XboxController(0);
    private XboxController assistant = new XboxController(1);
    private XboxController switchBoard = new XboxController(2);

    // TESTING Started in TestInit
    Command testWristCmd;
    TestArmRateCmd testArmCmd;

    public RobotTest() {
        OI();
        // TESTING Commands, only get scheduled if we enter Test mode
        testWristCmd = new  TestWristRateCommand();
                            //TestWristPositionCommand();
    }

    public void initialize() {
        // remove defaultCommands so only testing is being done.
        Robot.intake.setDefaultCommand(null);
        Robot.gearShifter.setDefaultCommand(null);
        Robot.arm.setDefaultCommand(null);
       
        // testArmCmd.start();
        testWristCmd.start();
    }

    public void periodic() {

    }

    /**
     * Bind any testing buttons/controls here
     */
    public void OI() {
        //Vacuum subsystem
        new JoystickButton(assistant, XboxControllerButtonCode.A.getCode()).whenPressed(new VacuumCommand(true));
        new JoystickButton(assistant, XboxControllerButtonCode.B.getCode()).whenPressed(new VacuumCommand(false));

        new JoystickButton(assistant, XboxControllerButtonCode.X.getCode()).whenPressed(new DownShiftCommand());
        new JoystickButton(assistant, XboxControllerButtonCode.Y.getCode()).whenPressed(new UpShiftCommand());
    }

}