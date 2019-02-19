package frc.robot.commands;

import java.util.function.IntSupplier;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.input.XboxControllerButtonCode;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.MoveArmAtHeight;
import frc.robot.commands.arm.MoveDownToCapture;
import frc.robot.commands.intake.VacuumCommand;
import frc.robot.commands.intake.WristTrackFunction;

/**
 * One class, singleton, to rule them all. Coordinates major modes of operation.
 * Handles tracking which commands get set when.
 * 
 */
public class CommandManager {
    int logCnt=0;

    // OI - operator inputs
    JoystickButton huntSelect; // used in hunting modes
    JoystickButton heightSelect; // used in delivery modes
    JoystickButton captureRelease; // used in delivery modes to go back to hunting

    // Button Commands
    Command huntSelectCmd;
    Command heightSelectCmd;
    Command captRelCmd;

    // Modes of behavior
    public enum Modes {
        Construction(0),  // system still coming up... not operational
        SettingZeros(1),  // calling all encoder power on requirements, not operational
        // Operational modes
        HuntGameStart(2), // special hunting piece mode - hunt the one on our robot
        HuntingHatch(3),  // Button:HuntSelect order 3->4->5--3-->4-->5...
        HuntingCargo(4),  // HuntSelect
        HuntingFloor(5),  // HuntSelect
        // Capture
        Capturing(6),     // moving from hunting to picking it up. Button:CaptureRelease
        // TODO: Make sure capturing leads right to recapturing, and make recapturing lead to deliver (if not dropped), hunting/capturing otherwise
        Recapturing(7),   // 3-second period where robot will be able to recapture hatch/cargo if dropped
        // DeliveryModes
        DeliverHatch(8),  // based on what we captured
        DeliverCargo(9),  // based on what we captured
        Ejecting(10);     // Button:CaptureRelease

        private int v;

        Modes(int v) {
            this.v = v;
        }

        public int get() {
            return v;
        }
    }

    enum PayLoad {
        Nothing, // inital state, return here after eject
        Cargo, // driven by sensor
        Hatch, // hard to tell, current on vacuum?
    }

    // Command Sets
    CommandGroup zeroRobotGrp;
    CommandGroup huntingHatchGrp;
    CommandGroup huntingCargoGrp;
    CommandGroup huntingHFloorGrp;
    CommandGroup captureGrp;
    CommandGroup deliveryGrp;
    CommandGroup ejectGrp;
    
    // Target States - think of this as desired command vector
    Modes currentMode; // what we think are doing now
    Modes prevHuntMode;
    Modes prevMode;

    CommandGroup currentGrp; // what is running

    double gripperH_cmd; // (inches) composite of arm/extender/wrist/cup

    // internal states
    int delHeightIdx = 0; // used in delivery selection
    int huntHeightIdx = 0;
    final Modes huntingModes[] = { Modes.HuntingHatch, Modes.HuntingCargo, Modes.HuntingFloor };
    int huntModeIdx = 0 ;  //starts at Hatch

    // Data points - shares delheightidx, must be same length
    final double DeliveryCargoHeights[] = { 32.0, 60.0, 88.0 }; // TODO: fix the numbers
    final double DeliveryHatchHeights[] = { 28.0, 56.0, 84.0 }; // TODO: fix the numbers
    final double Capture_dDown = 2.0;  //inches to move down for capture
    final double HuntHeights[] = { 28.0, 17.0, Capture_dDown + 4.0 }; // height from floor, H,C,Floor TODO:fix numbers

    public CommandManager() {
        currentMode = Modes.Construction;
        XboxController aCtlr = Robot.m_oi.getAssistantController();
        // XboxController dCtlr = Robot.m_oi.getDriverController();

        // setup buttons
        huntSelect = new JoystickButton(aCtlr, XboxControllerButtonCode.LB.getCode());
        heightSelect = new JoystickButton(aCtlr, XboxControllerButtonCode.TRIGGER_LEFT.getCode());
        captureRelease = new JoystickButton(aCtlr, XboxControllerButtonCode.TRIGGER_RIGHT.getCode());

        // define commands - bind local functions to be used on button hits
        huntSelectCmd = new CycleHuntModeCmd();         // (this::cycleHuntMode);
        heightSelectCmd = new CycleHeightModeCmd();
        captRelCmd = new CaptureReleaseCmd();
        
    
        // bind commands to buttons
        huntSelect.whenPressed(huntSelectCmd);
        heightSelect.whenPressed(heightSelectCmd);
        captureRelease.whenPressed(captRelCmd);

        // Construct our major modes
        zeroRobotGrp = CmdFactoryZeroRobot();
        huntingHFloorGrp = CmdFactoryHuntHatchFloor();
        huntingHatchGrp = CmdFactoryHuntHatch();
        huntingCargoGrp = CmdFactoryHuntCargo();
        captureGrp = CmdFactoryCapture();
        deliveryGrp = CmdFactoryDelivery();
        ejectGrp = CmdFactoryEject();
    }

    /**
     *   Handle the state transitions, set any state vars and setup next command group.
     *
     *   Gripper height setting is controlled by state changes while hunting so it 
     *   set in the here in setMode(). It uses the gripperHeight() to deliver the value
     *   to the running commands.
     * 
     *   When in delivery mode, the height is calculated by cycling through an array
     *   of heights.  The array index is bumpped on heightSelect button. The value is
     *   delivered to the commands via deliverGripperHeight();
     * 
     *   Different MoveArmAtHeight commands instantiated with the different gripperHeight
     *   functions assigned.
     */
    public void setMode(Modes mode) {
        CommandGroup nextCmd=null;
    
        switch (mode) {
        case SettingZeros:
            nextCmd = zeroRobotGrp;
            break;

        case HuntingFloor:
            gripperH_cmd = HuntHeights[2];
            nextCmd = huntingHFloorGrp;
            break;

        case HuntingCargo:
            gripperH_cmd = HuntHeights[1];
            nextCmd = huntingCargoGrp;
            break;

        case HuntingHatch:
            // move the heigh
            gripperH_cmd = HuntHeights[0];
            nextCmd = huntingHatchGrp;
            break;

        case Capturing: // moving from hunting to picking it up. Button:Capture
            prevHuntMode = currentMode;         //this is what we captured
            nextCmd = captureGrp;
            break;

        // DeliveryModes
        case DeliverHatch: // based on what we captured
            delHeightIdx = 0;
            gripperH_cmd = DeliveryHatchHeights[delHeightIdx];
            break;

        case DeliverCargo: // based on what we captured
            delHeightIdx = 0;
            gripperH_cmd = DeliveryCargoHeights[delHeightIdx];
            break;

        case Ejecting:
            nextCmd = ejectGrp;
            break;
        default:
            break;
        }
        // update states and start command group
        prevMode = currentMode;
        currentMode = mode;
        installGroup(nextCmd);
}
    //Starts new group. 
    void installGroup(CommandGroup grp) {
        if (grp == null) return;
        //do we need this???### currentGrp.cancel(); // end methods are called
        currentGrp = grp;
        currentGrp.start(); // schedule our new work, initialize() then execute() are called
    }

    public boolean isHunting() {
        if ((currentMode.get() > Modes.HuntGameStart.get()) && (currentMode.get() < Modes.Capturing.get())) {
            return true;
        }
        return false;
    }

    private void triggerCaptureRelease(){
        //Change StateMachine in command Manger
        setMode(Modes.Capturing);
    }
    private void cycleHuntMode() {
        
        if (isHunting()) {
            int idx = huntModeIdx + 1;
            huntModeIdx = (idx >= huntingModes.length) ? 0 : idx;
            setMode(huntingModes[huntModeIdx]);
        }
        // not hunting just ignore event
    }

    private int gotoDeliverMode() {
        //prevHunt mode saved on entering capturing mode... use it for hatch v cargo delivery
        Modes nextMode = (prevHuntMode == Modes.HuntingCargo) ? Modes.DeliverCargo : Modes.DeliverHatch;
        setMode(nextMode);
        return (nextMode.get());
    }

    private int gotoHuntMode() {
        Modes nextMode = Modes.HuntingHatch;
        return  nextMode.get();
    }

    private void cycleHeight() {
        if (isHunting()) return;  // if we are not delivery, just bail
        int idx = delHeightIdx + 1; // next height
        // make sure index fits in array
        delHeightIdx = (idx >= DeliveryCargoHeights.length) ? 0 : idx;
    }

    Double wristTrackParallel() {
        double phi = Robot.arm.getAngle();
        return (phi - 90.0);
    }

    Double wristTrackPerp() {
        //TODO: will need to account for phi on each side
        double phi = Robot.arm.getAngle();
        return (phi - 180.0);
    }

    // expose desired cup height to commands, set griperheight via state machine.
    Double gripperHeight() {
        return gripperH_cmd;
    }

    Double deliverGripperHeight() {
         gripperH_cmd =(prevHuntMode == Modes.HuntingCargo) ? 
            DeliveryCargoHeights[delHeightIdx] : DeliveryHatchHeights[delHeightIdx];
        return gripperH_cmd;
    }

    // Command Factories that build command sets for each mode of operation
    // These are largely interruptable so we can switch as state changes
    private CommandGroup CmdFactoryZeroRobot() {
        CommandGroup grp = new CommandGroup("ZeroRobot");
        grp.addParallel(Robot.arm.zeroSubsystem());
        grp.addParallel(Robot.intake.zeroSubsystem());

        // commands to come
        /// grp.addParallel(Robot.climber.zeroSubsystem());
        /// grp.addParallel(Robot.cargoTrap.zeroSubsystem());
        return grp;
    }

    private CommandGroup CmdFactoryHuntHatch() {
        CommandGroup grp = new CommandGroup("HuntHatch");
        grp.addParallel(new MoveArmAtHeight(this::gripperHeight));
        grp.addParallel(new WristTrackFunction(this::wristTrackParallel));
        return grp;
    }

    private CommandGroup CmdFactoryHuntCargo() {
        CommandGroup grp = new CommandGroup("HuntCargo");
        grp.addParallel(new MoveArmAtHeight(this::gripperHeight));
        grp.addParallel(new WristTrackFunction(this::wristTrackPerp));
        return grp;
    }

    private CommandGroup CmdFactoryHuntHatchFloor() {
        CommandGroup grp = new CommandGroup("HuntHatchFloor");
        grp.addParallel(new MoveArmAtHeight(this::gripperHeight));
        grp.addParallel(new WristTrackFunction(this::wristTrackPerp));
        return grp;
    }

    // Hunt Game Start will have a game piece at a starting place
    // that will require some special motion, turn on vacuum,
    // and finaly go into delivery.
    //
    private CommandGroup CmdFactoryHuntGameStart() {
        CommandGroup grp = new CommandGroup("HuntGameStart");
        return grp;
    }

    private CommandGroup CmdFactoryCapture() {
        CommandGroup grp = new CommandGroup("Capture");
        grp.addSequential(new VacuumCommand(true));                 /// new IntakeOnCommand() );    
        grp.addSequential(new MoveDownToCapture(Capture_dDown), 3.5 );  //TODO: fix  3.5 seconds const
        grp.addSequential(new CallFunctionCmd(this::gotoDeliverMode));
        return grp;
    }

    private CommandGroup CmdFactoryDelivery() {
        CommandGroup grp = new CommandGroup("Deliver");
        grp.addParallel(new MoveArmAtHeight(this::deliverGripperHeight));  //use deliver gripper funct
        grp.addParallel(new WristTrackFunction(this::wristTrackParallel));
        return grp;
    }

    private CommandGroup CmdFactoryEject() {
        CommandGroup grp = new CommandGroup("Eject");
        grp.addSequential(new VacuumCommand(false)); 
        grp.addSequential(new CallFunctionCmd(this::gotoHuntMode));
        return grp;
    }

    class CycleHuntModeCmd extends InstantCommand {
        
        @Override
        protected void execute() {
            cycleHuntMode();
        }
    }

    class CycleHeightModeCmd extends InstantCommand {
        
        @Override
        protected void execute() {
            cycleHeight();
        }
    }

    class CaptureReleaseCmd extends InstantCommand {
        
        @Override
        protected void execute() {
            triggerCaptureRelease();
        }
    }

    class CallFunctionCmd extends InstantCommand {
        IntSupplier workFunct;

        public CallFunctionCmd(IntSupplier workFunct) {
            this.workFunct = workFunct;
        }

        @Override
        public void execute() {
            workFunct.getAsInt();
        }
    }
    
    public void log() {
        logCnt++;
        if (logCnt % 50 ==0 ) {
            System.out.println("CmdMod:"+currentMode);
        }
    }

}
