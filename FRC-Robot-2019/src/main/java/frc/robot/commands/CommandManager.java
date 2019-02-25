package frc.robot.commands;

import java.util.function.IntSupplier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem.Position;
import frc.robot.commands.arm.MoveArmAtHeight;
import frc.robot.commands.arm.MoveDownToCapture;
import frc.robot.commands.intake.VacuumCommand;
import frc.robot.commands.intake.WristTrackFunction;
import frc.robot.commands.intake.*;
import frc.robot.commands.arm.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.util.RateLimiter;
import frc.robot.commands.util.RateLimiter.InputModel;
import frc.robot.commands.arm.tests.TestRotateArmToAngleCommand;

/**
 * One class, singleton, to rule them all. Coordinates major modes of operation.
 * Handles tracking which commands get set when.
 * 
 */
public class CommandManager {
    int logCnt=0;
    private long logTimer;

    
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
    CommandGroup huntGameStartGrp;
    CommandGroup huntingHatchGrp;
    CommandGroup huntingCargoGrp;
    CommandGroup huntingHFloorGrp;
    CommandGroup captureGrp;
    CommandGroup deliveryGrp;
    CommandGroup ejectGrp;
    
    // Target States - think of this as desired command vector
    Modes currentMode; // what we think are doing now
    Modes prevHuntMode = Modes.HuntingHatch;
    Modes prevMode;

    CommandGroup currentGrp; // what is running

    //gripper commanded postion - main output of the controls
    //RateLimiter rp_h;
    RateLimiter rr_ext;
    double gripperE_cmd = 0.0;    // (inches) Extension of arm/extender/wrist/cup
    double gripperH_cmd = 0.0;    // (inches) composite of arm/extender/wrist/cup

    // internal states
    int delHeightIdx = 0; // used in delivery selection
    int huntHeightIdx = 0;
    final Modes huntingModes[] = { Modes.HuntingHatch, Modes.HuntingCargo, Modes.HuntingFloor };
    int huntModeIdx = 0 ;  //starts at Hatch

    // Initial gripper projections by huntModeIdx
    // Extension from pivot point.
    final double projection[] = {22.0, 23.0, 22.0};  //TODO: fix the numbers
    
    // Data points - shares delheightidx, must be same length
    final double DeliveryCargoHeights[] = { 32.0, 60.0, 88.0 }; // TODO: fix the numbers
    final double DeliveryHatchHeights[] = { 28.0, 56.0, 84.0 }; // TODO: fix the numbers
    final double Capture_dDown = 5.0;  //inches to move down for capture
    final double HuntHeights[] = { 23.0, 16.0, Capture_dDown + 4.0 }; // height from floor, H,C,Floor TODO:fix numbers

    //Phyical values from sub-systems as needed
    Position armPosition;

    public CommandManager() {
        currentMode = Modes.Construction;
        // XboxController dCtlr = Robot.m_oi.getDriverController();
        // bind commands to buttons
        Robot.m_oi.huntSelect.whenPressed(new CycleHuntModeCmd());
        Robot.m_oi.heightSelect.whenPressed(new CycleHeightModeCmd());
        Robot.m_oi.captureRelease.whenPressed(new CaptureReleaseCmd());

        // Construct our major modes from their command factories
        zeroRobotGrp = CmdFactoryZeroRobot();
        huntGameStartGrp = CmdFactoryHuntGameStart();
        huntingHFloorGrp = CmdFactoryHuntHatchFloor();
        huntingHatchGrp = CmdFactoryHuntHatch();
        huntingCargoGrp = CmdFactoryHuntCargo();
        captureGrp = CmdFactoryCapture();
        deliveryGrp = CmdFactoryDelivery();
        ejectGrp = CmdFactoryEject();

        logTimer = System.currentTimeMillis();
        armPosition = Robot.arm.getArmPosition();

        rr_ext = new RateLimiter(Robot.dT, 
            Robot.m_oi::extensionInput,
            this::armExtension, null,
            Robot.arm.EXTEND_MIN,
            Robot.arm.EXTEND_MAX,
            -10.0, //inches/sec
            10.0, //inches/sec 
            InputModel.Rate);
        rr_ext.setDeadZone(.5);
        rr_ext.setRateGain(15.0);  // 15 in/sec max stick input 
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
        case Construction:
            break;
            
        case SettingZeros:
            nextCmd = zeroRobotGrp;
            break;

        case HuntGameStart:
            prevHuntMode = Modes.HuntingHatch;  // change this if we start with Cargo
            nextCmd = huntGameStartGrp;
            break;

        case HuntingFloor:
            huntModeIdx = 2;
            nextCmd = huntingHFloorGrp;
            break;

        case HuntingCargo:
            huntModeIdx = 1;
            nextCmd = huntingCargoGrp;
            break;

        case HuntingHatch:
            // move the height
            huntModeIdx = 0;
            nextCmd = huntingHatchGrp;
            break;

        case Capturing: // moving from hunting to picking it up. Button:Capture
            prevHuntMode = currentMode;         //this is what we captured
            nextCmd = captureGrp;
            break;

        // DeliveryModes
        case DeliverHatch: // based on what we captured
            delHeightIdx = 0;
            break;

        case DeliverCargo: // based on what we captured
            delHeightIdx = 0; 
            break;

        case Ejecting:
            nextCmd = ejectGrp;
            break;

        default:
            break;
        }
        // calculate the height and extension, set gripperH_cmd, and gripperE_cmd
        setGripperPosition();

        // update states and start command group
        prevMode = currentMode;
        currentMode = mode;
        installGroup(nextCmd);
}
    //Starts new group. 
    void installGroup(CommandGroup grp) {
        if (grp == null) return;
        if (currentGrp != null) currentGrp.cancel();   // end methods are called
        currentGrp = grp;
        currentGrp.start();    // schedule our new work, initialize() then execute() are called
    }

    public boolean isHunting() {
        if ((currentMode.get() > Modes.HuntGameStart.get()) && (currentMode.get() < Modes.Capturing.get())) {
            return true;
        }
        return false;
    }

    public boolean isDelivering() {
        return ( (currentMode == Modes.DeliverCargo)  || 
                 (currentMode == Modes.DeliverHatch) );
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

    // Select proper delivery mode based on what we were hunting.
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

    double wristTrackParallel() {
        double phi = Robot.arm.getAngle();
        return (phi - 90.0);
    }

    double wristTrackPerp() {
        //TODO: will need to account for phi on each side
        double phi = Robot.arm.getAngle();
        return (phi - 180.0);
    }

    /** 
     * setGripperPositon()
     * 
     *  Called on mode change to set arm location. Sets the commanded height and extension
     *  for the gripper when the mode changes.
     * 
     * */
    void setGripperPosition() {
        if (isHunting()) {
            //Hunting, use the HuntHeights table and that height index
            gripperH_cmd =  HuntHeights[huntHeightIdx];
            rr_ext.setState(projection[huntHeightIdx]); 
        }
        else if (isDelivering()) {
            //Delivering 
            gripperH_cmd = (prevHuntMode == Modes.HuntingCargo) ? 
                DeliveryCargoHeights[delHeightIdx] : DeliveryHatchHeights[delHeightIdx];
            rr_ext.setState(projection[0]); 
        }
        // Other mode changes just stay where we ar at
    }
    
    // expose desired cup height to commands, set griperheight via state machine.
    double gripperHeight() {
        return gripperH_cmd;
    }

    public double gripperExtension() {
        double ext = rr_ext.get();    // rate & postion limited
        return ext;
    }

    // called every frame, reads inputs 
    public void execute() {
        armPosition = Robot.arm.getArmPosition();
        //read inputs
        double capHeightIn = Robot.m_oi.captureHeightInput();  //trigger
        rr_ext.execute();
    }



    /**
     *  initialize the commands from the current position, sets the
     *  gripper[EH]_cmd to where they are right now. 
     * 
     *  Return only needed because it's invokeds as a FunctionCommand
     */
    int  initialize() {
        armPosition = Robot.arm.getArmPosition();   //update position
        gripperE_cmd = armPosition.projection;
        gripperH_cmd = armPosition.height;
        rr_ext.initialize();
        rr_ext.setState(armPosition.projection);
        return 0;
    }

    double armExtension() {
        return armPosition.projection;
    }
    

    // Command Factories that build command sets for each mode of operation
    // These are largely interruptable so we can switch as state changes
    private CommandGroup CmdFactoryZeroRobot() {
        CommandGroup grp = new CommandGroup("ZeroRobot");
        grp.addSequential(Robot.arm.zeroSubsystem());
        grp.addSequential(Robot.intake.zeroSubsystem());
        grp.addSequential(new CallFunctionCmd(this::initialize));

        // commands to come
        /// grp.addParallel(Robot.climber.zeroSubsystem());
        /// grp.addParallel(Robot.cargoTrap.zeroSubsystem());

        grp.addSequential(new NextModeCmd(Modes.HuntGameStart));
        return grp;
    }

    private CommandGroup CmdFactoryHuntHatch() {
        CommandGroup grp = new CommandGroup("HuntHatch");
        grp.addParallel(new MoveArmAtHeight(this::gripperHeight, this::gripperExtension));
        grp.addParallel(new WristTrackFunction(this::wristTrackParallel));
        return grp;
    }

    private CommandGroup CmdFactoryHuntCargo() {
        CommandGroup grp = new CommandGroup("HuntCargo");
        grp.addParallel(new MoveArmAtHeight(this::gripperHeight, this::gripperExtension));
        grp.addParallel(new WristTrackFunction(this::wristTrackPerp));
        return grp;
    }

    private CommandGroup CmdFactoryHuntHatchFloor() {
        CommandGroup grp = new CommandGroup("HuntHatchFloor");
        grp.addParallel(new MoveArmAtHeight(this::gripperHeight, this::gripperExtension));
        grp.addParallel(new WristTrackFunction(this::wristTrackPerp));
        return grp;
    }

    // Hunt Game Start will have a game piece at a starting place
    // that will require some special motion, turn on vacuum,
    // and finaly go into delivery.
    //
    private CommandGroup CmdFactoryHuntGameStart() {
        CommandGroup grp = new CommandGroup("HuntGameStart");
        grp.addSequential(new VacuumCommand(true));
        grp.addSequential(new RotateWristCommand(90.0, 2.0));              //these will wait, not timeout, 
        grp.addSequential(new RotateWristCommand(80.0, 2.0));              // wiggle wrist to grab hatch
        grp.addSequential(new RotateWristCommand(95.0, 2.0));
        grp.addSequential(new ExtendArmToPositionCommand(2.0));            //pull in a bit before rotate, should have hatch
        grp.addSequential(new TestRotateArmToAngleCommand(145.0, 30.0));   //rotate clear before going to delivery mode

        grp.addSequential(new NextModeCmd(Modes.DeliverHatch));
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
        grp.addParallel(new MoveArmAtHeight(this::gripperHeight, this::gripperExtension)); //use deliver gripper funct
        grp.addParallel(new WristTrackFunction(this::wristTrackParallel));
        return grp;
    }

    private CommandGroup CmdFactoryEject() {
        CommandGroup grp = new CommandGroup("Eject");
        grp.addSequential(new VacuumCommand(false)); 
        grp.addSequential(new CallFunctionCmd(this::gotoHuntMode));
        return grp;
    }

    //Used at the end of a command group to jump to next mode
    class NextModeCmd extends InstantCommand {
        Modes mode2set;
        NextModeCmd(Modes m) {
            mode2set = m;
        }

        @Override
        protected void execute() {
            setMode(mode2set);
        }
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
    
    public void log(int interval){
        if ((logTimer + interval) < System.currentTimeMillis()) { //only post to smartdashboard every interval ms
            logTimer = System.currentTimeMillis();
            SmartDashboard.putNumber("Command Mode", currentMode.get());
        }
    }

    /*  changed to new log format JR
    public void log() {
        logCnt++;
        if (logCnt % 50 ==0 ) {
            System.out.println("CmdMod:"+currentMode);
        }
    }
    */

}
