package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.input.XboxControllerButtonCode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GearShifterSubsystem;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.arm.TeleopArmControlCommand;
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
    private Command armTest;

    public RobotTest() {
        OI();
        // TESTING Commands, only get scheduled if we enter Test mode
        testWristCmd = new  TestWristRateCommand();
                            //TestWristPositionCommand();
        armTest = new TeleopArmControlCommand(this::leftJoyY, this::rightJoyY);
    }

    public void initialize() {
        // remove defaultCommands so only testing is being done.
        Robot.intake.setDefaultCommand(null);
        Robot.gearShifter.setDefaultCommand(null);
        Robot.arm.zeroArm();
        
        armTest.start();
        // testArmCmd.start();
        testWristCmd.start();
    }

    public void periodic() {
        logSmartDashboardSensors();
    }

    private double leftJoyY() {
        return Math.abs(assistant.getY(Hand.kLeft)) < 0.05? 0: -assistant.getY(Hand.kLeft);
    }

    private double rightJoyY() {
        return Math.abs(assistant.getY(Hand.kRight)) < 0.05? 0: -assistant.getY(Hand.kRight); 
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
    private void logSmartDashboardSensors() {
        // SmartDashboard.putNumber("Left Encoder Count", driveTrain.getLeftEncoderTalon().getSelectedSensorPosition());
        // SmartDashboard.putNumber("Left Encoder Rate", driveTrain.getLeftEncoderTalon().getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Right Encoder Count", driveTrain.getRightEncoderTalon().getSelectedSensorPosition());
        // SmartDashboard.putNumber("Right Encoder Rate", driveTrain.getRightEncoderTalon().getSelectedSensorVelocity());
        // SmartDashboard.putString("Gear Shifter State", String.valueOf(gearShifter.getCurGear()));

        Robot.arm.log();
        Robot.arm.logTalons();
 
        SmartDashboard.putData(Scheduler.getInstance()); 
        //SmartDashboard.putData(driveTrain);
        //SmartDashboard.putData(gearShifter);
      }
    
      private void resetAllDashBoardSensors() {
        Robot.driveTrain.getLeftEncoderTalon().setSelectedSensorPosition(0);
        Robot.driveTrain.getRightEncoderTalon().setSelectedSensorPosition(0);
      }
}