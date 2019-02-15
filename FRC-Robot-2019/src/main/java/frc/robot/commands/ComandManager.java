package frc.robot.commands;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

/**
 * One class, singleton, to rule them all.  Coordinates major modes of operation.
 * Handles tracking which commands get set when.
 * 
 */
public class ComandManager {
    Robot  r;          //short hand access to our static Robot structure

    //OI - operator inputs 
    Button HuntSelect;       //used in hunting modes
    Button HeightSelect;     //used in delivery modes
    Button Release;          //used in delivery modes to go back to hunting

    // Command Sets 
    CommandGroup zeroRobot;

    // Modes of behavior 
    enum Modes {
        Construction,       // system still coming up... not operational
        SettingZeros,       // calling all encoder power on requirements, not operational
        // Operational modes
        HuntGameStart,      // special hunting piece mode - hunt the one on our robot
        HuntingHatch,       // Button:HuntSelect
        HuntingCargo,       //        HuntSelect
        HuntingFloor,       //        HuntSelect
        // Capture
        Capturing,          // moving from hunting to picking it up. Button:Capture
        //DeliveryModes
        DeliverHatch,       // based on what we captured
        DeliverCargo,       // based on what we captured
        Ejecting,           // Button:Capture
    }

    enum PayLoad {
        Nothing,       // inital state, return here after eject
        Cargo,         // driven by sensor
        Hatch,         // hard to tell, current on vacuum?
    }

    // Target States - think of this as desired command vector
    Modes  currentMode;       // what we think are doing now
    double griperHeight;      // (inches) composite of arm/extender/wrist/cup
    

    // Data points
    final double DeliveryCargoHeights[] = { 24.0, 48.0, 78.0 };     //TODO: fix the numbers
    final double DeliveryHatchHeights[] = { 20.0, 42.0, 72.0 };      //TODO: fix the numbers

    public ComandManager(Robot r) {
        currentMode = Modes.Construction;
        this.r = r;

        // Construct our major modes
        zeroRobot = CmdFactoryZeroRobot();
    }

    // Command Factories that build command sets for each mode of operation
    // These are largely interruptable so we can switch as state changes
    private CommandGroup CmdFactoryZeroRobot() {
        CommandGroup grp = new CommandGroup();
        //grp.addParallel(r.arm.getInitCmd());
        return grp;
    }

}
