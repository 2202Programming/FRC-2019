package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.cargo.AutoTrapCargoCommand;
/**
 * Author: Alexander Ge
 *         Billy Haung
 *         Derek Laufenberg
 *  
 * Changes:  
 * 2/10/2019   DPL  Added constantants, improved naming conventions.
 *                  Use DoubleSolenoid.
 *                  
 */

/**
 * This is still TBD, we may use a roller style.
 * 
 */
public class CargoTrapSubsystem extends Subsystem {
    
    static final DoubleSolenoid.Value kOpenArms = Value.kForward;
    static final DoubleSolenoid.Value kCloseArms = Value.kReverse;  //off may work too
    
    static final DoubleSolenoid.Value kDeploy = Value.kForward;
    static final DoubleSolenoid.Value kRetract = Value.kReverse;  

    // Physical Devices and Controls
    DoubleSolenoid armsPiston = new DoubleSolenoid(RobotMap.TRAP_PCM_ID,
        RobotMap.TRAP_ARMS_OPEN_PCM, RobotMap.TRAP_ARMS_CLOSE_PCM);
                                           
    DoubleSolenoid deployPiston = new DoubleSolenoid(RobotMap.TRAP_PCM_ID,
         RobotMap.TRAP_DEPLOY_PCM, RobotMap.TRAP_RETRACT_PCM);

    DigitalInput cargoSensor = new DigitalInput(RobotMap.TRAP_CARGO_SENSOR_DIO);


    public CargoTrapSubsystem() {
        retractTrap();
        closeTrapArms();

        addChild("CargoTrap", armsPiston);
        addChild("Trap Deployment Solenoid", deployPiston);
    }

    @Override
    public void initDefaultCommand() {
        //setDefaultCommand(new AutoTrapCargoCommand());
    }

    /**
     * Access for Commands.
     */
    public void closeTrapArms() {
        armsPiston.set(kCloseArms);
    }

    public void openTrapArms() {
        armsPiston.set(kOpenArms);
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