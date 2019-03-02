package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
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
    DoubleSolenoid ratchet = new DoubleSolenoid(RobotMap.CLIMB_RATCHET_PCM_ID, RobotMap.CLIMB_RATCHET_UP_PCM,
            RobotMap.CLIMB_RATCHET_DOWN_PCM);

    Spark footExtender = new Spark(RobotMap.CLIMB_FOOT_SPARK_PWM);
    Spark roller = new Spark(RobotMap.CLIMB_ROLLER_SPARK_PWM);

    //think we need to add an encoder
    int placeHolder1 = 0; //place holders for channels
    int placeHolder2 = 0;
    Encoder extensionEncoder = new Encoder(placeHolder1, placeHolder2);

    DoubleSolenoid drawerSlide = new DoubleSolenoid(placeHolder1, placeHolder2);

    public Encoder getExtentionEncoder()
    {
        return extensionEncoder;
    }

    public void setDrawerSlide(DoubleSolenoid.Value value)
    {
        drawerSlide.set(value);
    }

    void init() {
        roller.disable();
        footExtender.disable();
        ratchet.set(Retract);
    }

    public ClimberSubsystem() {
        init();
        addChild("Climber-ratchet", ratchet);
        addChild("Climber-foot", footExtender);
        addChild("Climber-roller", roller);
    }

    public void setExtenderSpeed(double speed) {
        footExtender.set(speed);
    }

    public double getExtenderSpeed() {
        return footExtender.get();
    }

    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    public double getRollerSpeed() {
        return roller.get();
    }

    public void setRatchetExtend(boolean extend) {
        if (extend)  { ratchet.set(Extend);  } 
        else         { ratchet.set(Retract); }
    }

    public boolean getRatchetExtend() {
        return (Extend == ratchet.get()); 
        }

    @Override
    protected void initDefaultCommand() {
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ClimberSubsystem");
        builder.addDoubleProperty("ExtenderSpeed", this::getExtenderSpeed, this::setExtenderSpeed);
        builder.addDoubleProperty("FootSpeed", this::getRollerSpeed, this::setRollerSpeed);
        builder.addBooleanProperty("RatchetExtend", this::getRatchetExtend, this::setRatchetExtend);
    }

    public void log() {
        SmartDashboard.putData(this);
    }

}