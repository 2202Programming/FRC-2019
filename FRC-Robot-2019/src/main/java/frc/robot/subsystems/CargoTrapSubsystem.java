package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotMap.*;

/**
 * This is still TBD, we may use a roller style.
 * 
 */
public class CargoTrapSubsystem extends SubsystemBase {
    
    static final DoubleSolenoid.Value kDeploy = Value.kForward;
    static final DoubleSolenoid.Value kRetract = Value.kReverse;  

    // Physical Devices and Controls
    Spark intakeMotors = new Spark(TRAP_INTAKE_MOTOR_PIN);
                                           
    DoubleSolenoid deployPiston = new DoubleSolenoid(TRAP_PCM_ID, 
         PneumaticsModuleType.CTREPCM,  TRAP_DEPLOY_PCM, TRAP_RETRACT_PCM);

    DigitalInput cargoSensor = new DigitalInput(TRAP_CARGO_SENSOR_DIO);


    public CargoTrapSubsystem() {
        retractTrap();
        addChild("Trap Deployment Solenoid", deployPiston);
    }

    //TODO - WIRE THIS IN  DPL 12/16/2021
    public void initDefaultCommand() {
        //setDefaultCommand(new AutoTrapCargoCommand());
        retractTrap();
    }

    /**
     * Access for Commands.
     */
    public void setIntake(double speed) {
        intakeMotors.set(speed);
    }

    public void deployTrap() {
        deployPiston.set(kDeploy);
    }

    public void retractTrap() {
        deployPiston.set(kRetract);
    }


    /**
     * Sensor Interface
     * @return
     */
    public boolean cargoInSight() {
        return cargoSensor.get();
    }
}