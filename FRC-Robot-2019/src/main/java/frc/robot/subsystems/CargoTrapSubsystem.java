package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.cargo.AutoTrapCargoCommand;

public class CargoTrapSubsystem extends Subsystem {
    public Solenoid trapArms = new Solenoid(RobotMap.TRAP_ARMS_PCM_ID);
    public Solenoid trapDeploy = new Solenoid(RobotMap.TRAP_RETRACT_PCM_ID);
   // public DigitalInput cargoDetector = new DigitalInput(RobotMap.CARGO_SENSOR_DIO_PORT);

    public CargoTrapSubsystem() {
        addChild("Trap Arms Solenoid", trapArms);
        addChild("Trap Deployment Solenoid", trapDeploy);
       // addChild("Cargo Detector", cargoDetector);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new AutoTrapCargoCommand());
    }

    public void closeTrapArms() {
        trapArms.set(true);
    }

    public void openTrapArms() {
        trapArms.set(false);
    }

    public void deployTrap() {
        trapDeploy.set(true);
    }

    public void retractTrap() {
        trapDeploy.set(false);
    }

   /*
    public boolean containsCargo() {
        return cargoDetector.get();
    }
    */
}