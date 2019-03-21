package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.arm.TeleopArmControlCommand;
import frc.robot.commands.arm.tests.TestArmRateCmd;
import frc.robot.commands.intake.tests.TestWristPositionCommand;


public class RobotTest {
    private XboxController driver = Robot.m_oi.getDriverController();
    private XboxController assistant = Robot.m_oi.getAssistantController();
    private XboxController switchBoard = Robot.m_oi.getAssistantController();

    // TESTING Started in TestInit
    Command testWristCmd;
    TestArmRateCmd testArmCmd;
    private Command armTest;

    public RobotTest() {
        System.out.println("IN TEST MODE");
    }

    public void initialize() {
        // Set commands here so they override the OI 
        Scheduler.getInstance().removeAll();
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
    private double Wrist_AssistLeftTrigger() {
        //rescale as expected by wrist test
        double temp = -1.0 +2.0*Robot.m_oi.getAssistantController().getTriggerAxis(Hand.kLeft);
        return temp;
    }

    private double leftJoyY() {
        return Math.abs(assistant.getY(Hand.kLeft)) < 0.05? 0: -assistant.getY(Hand.kLeft);
    }

    private double rightJoyY() {
        return Math.abs(assistant.getY(Hand.kRight)) < 0.05? 0: -assistant.getY(Hand.kRight); 
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