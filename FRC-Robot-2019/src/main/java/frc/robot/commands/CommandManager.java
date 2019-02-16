package frc.robot.commands;
import java.util.function.*;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.input.XboxControllerButtonCode;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.intake.WristTrackFunction;

/**
 * One class, singleton, to rule them all.  Coordinates major modes of operation.
 * Handles tracking which commands get set when.
 * 
 */
public class CommandManager {
    //OI - operator inputs 
    JoystickButton huntSelect;       //used in hunting modes
    JoystickButton heightSelect;     //used in delivery modes
    JoystickButton captureRelease;   //used in delivery modes to go back to hunting

    // Button Commands
    Command huntSelectCmd;
    Command heightSelectCmd;
    Command captRelCmd;

    
    // Modes of behavior 
    public enum Modes {
        Construction(0),       // system still coming up... not operational
        SettingZeros(1),       // calling all encoder power on requirements, not operational
        // Operational modes
        HuntGameStart(2),      // special hunting piece mode - hunt the one on our robot
        HuntingHatch(3),       // Button:HuntSelect order 3->4->5--3-->4-->5...
        HuntingCargo(4),       //        HuntSelect
        HuntingFloor(5),       //        HuntSelect
        // Capture
        Capturing(6),          // moving from hunting to picking it up. Button:Capture
        //DeliveryModes
        DeliverHatch(7),       // based on what we captured
        DeliverCargo(8),       // based on what we captured
        Ejecting(9);           // Button:Capture
        
        private int v;
        Modes(int v) { this.v = v;  }
        public int get() {return v;}
    }

    enum PayLoad {
        Nothing,       // inital state, return here after eject
        Cargo,         // driven by sensor
        Hatch,         // hard to tell, current on vacuum?
    }
    // Command Sets  
    CommandGroup zeroRobotGrp;
    CommandGroup huntingHatchGrp;
    CommandGroup huntingCargoGrp;
    CommandGroup huntingHFloorGrp;

    

    // Target States - think of this as desired command vector
    Modes  currentMode;       // what we think are doing now
    CommandGroup currentGrp; 
    Modes  prevMode;
    double griperHeight;      // (inches) composite of arm/extender/wrist/cup
    
    // internal states
    int  delHeightIdx = 0;      //used in delivery selection

    // Data points - shares delheightidx, must be same length
    final double DeliveryCargoHeights[] = { 24.0, 48.0, 78.0 };     //TODO: fix the numbers
    final double DeliveryHatchHeights[] = { 20.0, 42.0, 72.0 };     //TODO: fix the numbers
    
    public CommandManager() {
        currentMode = Modes.Construction;
        XboxController aCtlr = Robot.m_oi.getAssistantController();
        //XboxController dCtlr = Robot.m_oi.getDriverController();
        
        //setup buttons
        huntSelect = new JoystickButton(aCtlr, XboxControllerButtonCode.LB.getCode());
        heightSelect = new JoystickButton(aCtlr, XboxControllerButtonCode.TRIGGER_LEFT.getCode());
        captureRelease = new JoystickButton(aCtlr, XboxControllerButtonCode.TRIGGER_RIGHT.getCode());
        
        //define commands - bind local functions to be used on button hits
        huntSelectCmd = new CallFunctionCmd(this::cycleHuntMode);
        heightSelectCmd = new CallFunctionCmd(this::cycleHeight);
    
        //bind commands to buttons
        huntSelect.whenPressed(huntSelectCmd);
        heightSelect.whenPressed(heightSelectCmd);


        // Construct our major modes
        zeroRobotGrp = CmdFactoryZeroRobot();
        

    }

    //handle the state transitions
    public void setMode(Modes mode) {
       
        //TODO: need to do the work here to swithc modes..

        switch (mode) {
            case HuntingFloor:
            case HuntingCargo:
            case HuntingHatch:
            break;


            default:
        }
         prevMode = currentMode;
        currentMode = mode;
    }

     void installGroup(CommandGroup grp)
    {
    
    }

    public boolean isHunting() {
        if ((currentMode.get() > Modes.HuntGameStart.get()) && 
            (currentMode.get() < Modes.Capturing.get())) {
                return true;
        }
        return false;
    }


    private void cycleHuntMode(int unused) {
        final Modes ModeToMode[] = {Modes.HuntingCargo, Modes.HuntingCargo, Modes.HuntingFloor };
        if (isHunting()) {
            Modes newMode =  ModeToMode[(currentMode.get() - Modes.HuntingHatch.get())];
            setMode(newMode);
        }
        // not hunting just ignore event
    }

    private void cycleHeight(int unused) {
        int idx = delHeightIdx +1;  //next height
        // make sure index fits in array
        delHeightIdx = (idx > DeliveryCargoHeights.length) ?  0 : idx;
    }

    Double wristTrackParallel( ){
        double phi = Robot.arm.getAngle();
        return (phi -90.0);
    }

    Double wristTrackPerp() {
        double phi = Robot.arm.getAngle();
        return (phi -180.0);
    }

    // expose desired cup height to commands, set griperheight via state machine.
    Double cupHeight() {  return griperHeight; }

    // Command Factories that build command sets for each mode of operation
    // These are largely interruptable so we can switch as state changes
    private CommandGroup CmdFactoryZeroRobot() {
        CommandGroup grp = new CommandGroup("ZeroRobot");
        grp.addParallel(Robot.arm.zeroSubsystem());
        grp.addParallel(Robot.intake.zeroSubsystem());
        
        //commands to come
        ///grp.addParallel(Robot.climber.zeroSubsystem());
        ///grp.addParallel(Robot.cargoTrap.zeroSubsystem());
        return grp;
    }

    private CommandGroup CmdFactoryHuntHatch() {
        CommandGroup grp = new CommandGroup("HuntHatch");
        // ArmToHeight(getH)
        grp.addParallel( new WristTrackFunction(this::wristTrackParallel));

        return grp;
    }
    private CommandGroup CmdFactoryHuntCargo() {
        CommandGroup grp = new CommandGroup("HuntCargo");
        //ArmToHeight(getH)
        grp.addParallel(new WristTrackFunction(this::wristTrackPerp));

        return grp;
    }
    private CommandGroup CmdFactoryHuntHatchFloor() {
        CommandGroup grp = new CommandGroup("HuntHatchFloor");
        //ArmToHeight(getH)
        grp.addParallel(new WristTrackFunction(this::wristTrackPerp));
        return grp;
    }
    private CommandGroup CmdFactoryHuntGameStart() {
        CommandGroup grp = new CommandGroup("HuntGameStart");
        return grp;
    }
    
    private CommandGroup CmdFactoryCapture() {
        CommandGroup grp = new CommandGroup("Capture");
        return grp;
    }
    private CommandGroup CmdFactoryDeliverHatch() {
        CommandGroup grp = new CommandGroup("DeliverHatch");
        grp.addParallel(new WristTrackFunction(this::wristTrackParallel));
        return grp;
    }

    private CommandGroup CmdFactoryDeliverCargo() {
        CommandGroup grp = new CommandGroup("DeliverCargo");
        grp.addParallel(new WristTrackFunction(this::wristTrackParallel));
        return grp;
    }

   
    class CallFunctionCmd extends Command {
        IntConsumer workFunct;

        public CallFunctionCmd(IntConsumer workFunct) {
            this.workFunct = workFunct;
        }
        @Override
        public void initialize() {
            workFunct.accept(0);
        }

        @Override 
        public boolean isFinished() { return true;}
    }

}
