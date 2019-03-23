package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Authors: Derek Laufenberg
 *          Billy Huang
 *          Alexander Ge
 *         
 * Changes:  
 * 2/10/2019   DPL  Created first draft of climber subsystem
 *                  
 * 
 */

/**
 * Climber subsystem consistes of an extensible foot to lift and lower the robot
 * with. When the robot is lifted up, a motor on the foot is used to roll the
 * robot past the step so the robot can be lowered on the higher step.
 *
 * A gear/pawl mechanism driven by a solenoid prevents the foot from backsliding
 * when rasing or lowering.
 *
 */

public class ClimberSubsystem extends Subsystem {
    // constants
    // DPL ### check the settings for Extend/Retract
    final DoubleSolenoid.Value Extend = Value.kForward;
    final DoubleSolenoid.Value Retract = Value.kReverse;
    public final DoubleSolenoid.Value PullIn = Value.kReverse;
    public final DoubleSolenoid.Value Release = Value.kForward;

    public final double STALL_POWER_EXTEND = 0.069; //Power needed to allow pawl to fire while on the ground 
    public final double STALL_POWER_RETRACT = 0.420; //Power needed to allow pawl to fire while extended up
    public final double COUNTS_PER_IN = 150; //TODO: Find actual counts
    public final double IN_PER_COUNT = 1 / COUNTS_PER_IN;

    // physical devices
    DoubleSolenoid pawl = new DoubleSolenoid(RobotMap.CLIMB_PCM_ID, RobotMap.CLIMB_PAWL_ENGAGE_PCM,
            RobotMap.CLIMB_PAWL_RELEASE_PCM);
    DoubleSolenoid drawerSlide = new DoubleSolenoid(RobotMap.CLIMB_PCM_ID, RobotMap.CLIMB_SLIDE_PULL_PCM,
            RobotMap.CLIMB_SLIDE_RELEASE_PCM);

    CANSparkMax footExtender = new CANSparkMax(RobotMap.CLIMB_FOOT_SPARK_MAX_CAN_ID, MotorType.kBrushless);
    CANSparkMax roller = new CANSparkMax(RobotMap.CLIMB_ROLLER_SPARK_MAX_CAN_ID, MotorType.kBrushed);

    // think we need to add an encoder

    public void setDrawerSlide(boolean on) {
        if (on)
            drawerSlide.set(Extend);
        else
            drawerSlide.set(Retract);
    }

    public void setPawl( DoubleSolenoid.Value cmd) {
        pawl.set(cmd);
    }
    
    public void setPawl(boolean on) {
        if (on)
            pawl.set(PullIn);
        else
            pawl.set(Release);
    }

    public ClimberSubsystem() {
    }

    public void setExtenderSpeed(double speed) {
        footExtender.set(speed);
    }

    public double getExtenderSpeed() {
        return footExtender.get();
    }

    public double getExtension() {
        return footExtender.getEncoder().getPosition() * IN_PER_COUNT;
    }

    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    public double getRollerSpeed() {
        return roller.get();
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ClimberSubsystem");
        builder.addDoubleProperty("ExtenderSpeed", this::getExtenderSpeed, this::setExtenderSpeed);
        builder.addDoubleProperty("FootSpeed", this::getRollerSpeed, this::setRollerSpeed);
    }

    public void log() {
        SmartDashboard.putNumber("Climber counts", getExtension());
    }

}