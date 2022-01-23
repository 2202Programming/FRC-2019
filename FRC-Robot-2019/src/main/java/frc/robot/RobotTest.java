package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.arm.TeleopArmControlCommand;
import frc.robot.commands.arm.tests.TestArmRateCmd;

public class RobotTest {
    @SuppressWarnings("unused")
    private XboxController driver = Robot.m_oi.getDriverController();
    private XboxController assistant = Robot.m_oi.getAssistantController();
    @SuppressWarnings("unused")
    private XboxController switchBoard = Robot.m_oi.getAssistantController();

    // TESTING Started in TestInit
    Command testWristCmd;
    TestArmRateCmd testArmCmd;
    @SuppressWarnings("unused")
    private Command armTest;

    public RobotTest() {
    }

    public void initialize() {
        // Set commands here so they override the OI 
        CommandScheduler.getInstance().cancelAll();
        // remove defaultCommands so only testing is being done.
        Robot.intake.setDefaultCommand(null);
        Robot.gearShifter.setDefaultCommand(null);
        Robot.arm.zeroArm();

        // TESTING Commands, only get scheduled if we enter Test mode
        //testWristCmd = new  TestWristPositionCommand(this::Wrist_AssistLeftTrigger);
        armTest = new TeleopArmControlCommand(this::leftJoyY, this::rightJoyY);
        
        //armTest.start();
        //testWristCmd.start();
    }

    public void periodic() {
        logSmartDashboardSensors();
    }

    /**
     * 
     * Bind the Joystick control functions here and use DoubleSupplier function arguemnts to pass
     * them into your test functions. 
     * 
     * This keeps all the Joystick bindings out of the bowels of the code and signals can be 
     * modified as needed.  
     * 
     * 
     */
    @SuppressWarnings("unused")
    private double Wrist_AssistLeftTrigger() {
        //rescale as expected by wrist test
        double temp = -1.0 +2.0*Robot.m_oi.getAssistantController().getLeftTriggerAxis();
        return temp;
    }

    private double leftJoyY() {
        return Math.abs(assistant.getLeftY()) < 0.05? 0: -assistant.getLeftY();
    }

    private double rightJoyY() {
        return Math.abs(assistant.getRightY()) < 0.05? 0: -assistant.getRightY(); 
    }
       
    private void logSmartDashboardSensors() {
        // SmartDashboard.putNumber("Left Encoder Count", driveTrain.getLeftEncoderTalon().getSelectedSensorPosition());
        // SmartDashboard.putNumber("Left Encoder Rate", driveTrain.getLeftEncoderTalon().getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Right Encoder Count", driveTrain.getRightEncoderTalon().getSelectedSensorPosition());
        // SmartDashboard.putNumber("Right Encoder Rate", driveTrain.getRightEncoderTalon().getSelectedSensorVelocity());
        // SmartDashboard.putString("Gear Shifter State", String.valueOf(gearShifter.getCurGear()));

        Robot.arm.log(200);  // dpl - 1/23/2022 should be every 200ms log
        Robot.arm.logTalons();
 
        SmartDashboard.putData(CommandScheduler.getInstance()); 
        //SmartDashboard.putData(driveTrain);
        //SmartDashboard.putData(gearShifter);
      }
      
      @SuppressWarnings("unused")
      private void resetAllDashBoardSensors() {
        Robot.driveTrain.getLeftEncoderTalon().setSelectedSensorPosition(0);
        Robot.driveTrain.getRightEncoderTalon().setSelectedSensorPosition(0);
      }
}