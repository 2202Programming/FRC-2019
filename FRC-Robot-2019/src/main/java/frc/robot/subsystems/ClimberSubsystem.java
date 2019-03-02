package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.Sendable;
//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
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

    // physical devices
    Solenoid pawl = new Solenoid(RobotMap.CLIMB_PCM_ID, RobotMap.CLIMB_PAWL_PCM);
    Solenoid drawerSlide = new Solenoid(RobotMap.CLIMB_PCM_ID, RobotMap.CLIMB_SLIDE_PCM);

    CANSparkMax footExtender = new CANSparkMax(RobotMap.CLIMB_FOOT_SPARK_MAX_CAN_ID, MotorType.kBrushless);
    CANSparkMax roller = new CANSparkMax(RobotMap.CLIMB_ROLLER_SPARK_MAX_CAN_ID, MotorType.kBrushed);

    //think we need to add an encoder

    public void setDrawerSlide(boolean on)
    {
        drawerSlide.set(on);
    }

    public void setPawl(boolean on) {
        pawl.set(on);
    }

    void init() {
        roller.disable();
        footExtender.disable();
    }

    public ClimberSubsystem() {
        init();
        addChild("Climber-ratchet", pawl);
        addChild("Climber-slide", drawerSlide);
        addChild("Climber-foot", (Sendable) footExtender);
        addChild("Climber-roller", (Sendable) roller);
    }

    public void setExtenderSpeed(double speed) {
        footExtender.set(speed);
    }

    public double getExtenderSpeed() {
        return footExtender.get();
    }

    public double getExtension() {
        return footExtender.getEncoder().getPosition();
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
        SmartDashboard.putData(this);
    }

}