package frc.robot.commands;

import java.util.function.IntSupplier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem.Position;
import frc.robot.commands.arm.MoveArmAtHeight;
import frc.robot.commands.intake.VacuumCommand;
import frc.robot.commands.intake.WristTrackFunction;
import frc.robot.commands.intake.*;
import frc.robot.commands.arm.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.util.RateLimiter;
import frc.robot.commands.util.RateLimiter.InputModel;
import frc.robot.commands.arm.tests.TestRotateArmToAngleCommand;
import frc.robot.commands.util.MathUtil;


/**
 * One class, singleton, to rule them all. Coordinates major modes of operation.
 * Handles tracking which commands get set when.
 * 
 */
public class CommandManager {
    public final double kCapHeight = 4.0;  //inch/joy units  TODO: put in better place
    public final double kHeightMin = 2.0;    //inches
    public final double kHeightMax = 96.0;   //inches

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
        Capturing(6),        // TODO: UNUSED right now, 
        // TODO: Make sure capturing leads right to recapturing, and make recapturing lead to deliver (if not dropped), hunting/capturing otherwise
        Recapturing(7),     // UNUSED right now 3-second period where robot will be able to recapture hatch/cargo if dropped
        // DeliveryModes
        DeliverDriver(8),  // Unused, Richard suggest we tuck in with game piece until ready
        DeliverHatch(9),   // based on what we captured
        DeliverCargo(10),  // based on what we captured
        Releasing(20);     // Button:CaptureRelease

        private int v;

        Modes(int v) {
            this.v = v;
        }

        public int get() {
            return v;
        }
    }

    // Command Sets - one created for each major operation of the robot.
    CommandGroup zeroRobotGrp;
    CommandGroup huntGameStartGrp;
    CommandGroup huntingHatchGrp;
    CommandGroup huntingCargoGrp;
    CommandGroup huntingHFloorGrp;
    CommandGroup captureGrp;
    CommandGroup deliveryGrp;
    CommandGroup releaseGrp;
    
    // Target States - think of this as desired command vector
    Modes currentMode; // what we think are doing now
    Modes prevHuntMode = Modes.HuntingHatch;   // what we caught, used to tell if hatch or cargo 
    Modes prevMode;

    CommandGroup currentGrp; // what is running

    //gripper commanded postion - main output of the controls
    //RateLimiter rp_h;
    RateLimiter xprojRL;          // rate limited Xprojection output (inches)
    RateLimiter heightRL;         // rate limited height output (inches)
    // step command values used as inputs to RateLimiters, inches these get smoothed
    double gripperX_cmd = 0.0;    // (inches) Projection of arm/extender/wrist/cup
    double gripperH_cmd = 0.0;    // (inches) composite of arm/extender/wrist/cup

    // internal states
    int delHeightIdx = 0;         // used in Delivery<Cargo/Hatch>Heights[]

    // Data points - shares delheightidx, must be same length
    final double DeliveryCargoHeights[] = { 28.0, 56.0, 84.0 };  // TODO: fix the numbers
    final double DeliveryHatchHeights[] = { 24.0, 52.0, 81.5 };  // TODO: fix the numbers
    final double deliveryProjection[] =  {25.0, 25.0, 40.0};     //TODO: fix the numbers

    final Modes huntingModes[] = { Modes.HuntingFloor, Modes.HuntingCargo, Modes.HuntingHatch};
    final double HuntHeights[] = { 5.0, 17.5, 24.0 };    // height from floor, H,C,Floor TODO:fix numbers 
    final double huntProjection[] = {21.0, 22.0, 24.0};  //TODO: fix the numbers
    int huntModeIdx = 2;  //hatch

    //Phyical values from sub-systems as needed
    Position armPosition;

    public CommandManager() {
        currentMode = Modes.Construction;
        // bind commands to buttons
        Robot.m_oi.heightDownSelect.whenPressed(new CallFunctionCmd(this::cycleDown));
        Robot.m_oi.heightUpSelect.whenPressed(new CallFunctionCmd(this::cycleUp));
        Robot.m_oi.captureRelease.whenPressed(new CallFunctionCmd(this::triggerCaptureRelease));

        // Construct our major modes from their command factories
        zeroRobotGrp = CmdFactoryZeroRobot();
        huntGameStartGrp = CmdFactoryHuntGameStart();
        huntingHFloorGrp = CmdFactoryHuntHatchFloor();
        huntingHatchGrp = CmdFactoryHuntHatch();
        huntingCargoGrp = CmdFactoryHuntCargo();
        captureGrp = CmdFactoryCapture();
        deliveryGrp = CmdFactoryDelivery();
        releaseGrp = CmdFactoryRelease();

        logTimer = System.currentTimeMillis();
        armPosition = Robot.arm.getArmPosition();

        xprojRL = new RateLimiter(Robot.dT, 
            Robot.m_oi::extensionInput,    //inputFunc
            this::measProjection,          //phy position func
            Robot.arm.MIN_PROJECTION,      //output min
            Robot.arm.MAX_PROJECTION,      //output max
            -7.0, //inches/sec             // falling rate limit
             7.0,  //inches/sec            //raising rate limit
            InputModel.Rate);
        xprojRL.setDeadZone(0.2);   // ignore .2 in/sec on stick
        xprojRL.setRateGain(-10.0);  // -10 in/sec (neg is stick forward)

        heightRL = new RateLimiter(Robot.dT,
            this::get_gripperH_cmd,        // gripperH_cmd var as set by this module
            this::measHeight,              //phy position func
            kHeightMin,                    //output min
            kHeightMax,                    //output max
            -10.0,  //inches/sec            // falling rate limit
             10.0,  //inches/sec            //raising rate limit
            InputModel.Position);
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
            nextCmd = huntingHFloorGrp;
            break;

        case HuntingCargo:
            nextCmd = huntingCargoGrp;
            break;

        case HuntingHatch:
            huntModeIdx = 2;         
            nextCmd = huntingHatchGrp;
            break;

        case Capturing:         // moving from hunting to picking it up. Button:Capture
            prevHuntMode = currentMode;         //this is what we captured
            nextCmd = captureGrp;
            break;

        // DeliveryModes
        case DeliverHatch:            // based on what we captured
            delHeightIdx = 0;         //start at lowest
            nextCmd = deliveryGrp;
            break;

        case DeliverCargo:            // based on what we captured
            delHeightIdx = 0; 
            nextCmd = deliveryGrp;
            break;

        case Releasing:
            nextCmd = releaseGrp;
            break;

        default:
            break;
        }

        // update states and start command group
        prevMode = currentMode;
        currentMode = mode;
        // calculate the height and extension, set gripperH_cmd, and gripperE_cmd
        setGripperPosition();
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

    // called on capture/release trigger button
    private int triggerCaptureRelease() {
        if (isHunting())  
        {
            prevHuntMode = currentMode;   
            gotoDeliverMode();
        }
        else  
           setMode(Modes.Releasing);
        return 0;
    }


    /**************************************************************************************************
     *  Used for both hunting and delivering, called by LB/RB on assistant controller
     * @param direction  1--> go up   -1 --> go down 
     * @return
     */    
    private int cycleHeightMode(int direction) {

        if (isHunting()) {
            int idx = huntModeIdx + direction;
            prevHuntMode = huntingModes[huntModeIdx];
            huntModeIdx = MathUtil.limit(idx, 0, huntingModes.length -1);
            setMode(huntingModes[huntModeIdx]);
            return huntModeIdx;
        }
        else if (isDelivering()) {
            int idx = delHeightIdx + direction; // next height
            // make sure index fits in array
            delHeightIdx =  MathUtil.limit(idx, 0,  DeliveryCargoHeights.length -1);
            setGripperPosition();
            return delHeightIdx;
        }
        return(-1);
        //todo: drive delivery here???
    }
    // call the above code either going up or down, used by LB or RB 
    private int cycleUp()   { return cycleHeightMode(1); }
    private int cycleDown() { return cycleHeightMode(-1); }

    /******************************************************************************************** */

    // Select proper delivery mode based on what we were hunting.
    private int gotoDeliverMode() {
        //prevHunt mode saved on entering capturing mode... use it for hatch v cargo delivery
        Modes nextMode = (prevHuntMode == Modes.HuntingCargo) ? Modes.DeliverCargo : Modes.DeliverHatch;
        setMode(nextMode);
        return (nextMode.get());
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
     *  Called on mode change to set arm location. Sets the commanded height and projection
     *  for the gripper when the mode changes.  H, Proj are rate limited.
     * 
     * */
    void setGripperPosition() {
        if (isHunting()) {
            //Hunting, use the HuntHeights table and that height index
            gripperH_cmd = HuntHeights[huntModeIdx];
            gripperX_cmd = huntProjection[huntModeIdx];
            xprojRL.setForward(gripperX_cmd);      //xproj is rate controlled so it uses a forward val
        }
        else if (isDelivering()) {
            //Delivering 
            gripperH_cmd = (prevHuntMode == Modes.HuntingCargo) ? 
                DeliveryCargoHeights[delHeightIdx] : DeliveryHatchHeights[delHeightIdx];
            gripperX_cmd = deliveryProjection[delHeightIdx];
            xprojRL.setForward(gripperX_cmd);      //xproj is rate controlled so it uses a forward val
        }
        // Other mode changes just stay where we are at
    }
    
    /**************************************************************************************************************/
    /**
     * Expose desired gripper height & extension with double supplier functions
     */
    double gripperHeightOut() {
        return heightRL.get(); 
    }

    public double gripperXProjectionOut() {
        double xproj = xprojRL.get();    // rate & postion limited
        return xproj;
    }
    /************************************************************************************************************/

    // called every frame, reads inputs 
    public void execute() {
        armPosition = Robot.arm.getArmPosition();
        // allow the operator direct position offsets, no rate filtering as changes will be small
        
        //read inputs, apply rate limits to commands 
        xprojRL.execute();
        heightRL.execute();
    }

    // called by rateLimiter as input for height command to arm
    // Uses state machine and driver input
    private double get_gripperH_cmd() {
        double h_driverOffset = kCapHeight * Robot.m_oi.adjustHeight();  //driver contrib from triggers
        double h = gripperH_cmd - h_driverOffset;    //state machine + driver so both are rate filtered
        return h;
    }
    //private double get_gripperX_cmd() {return gripperX_cmd;} uses .setForward()

    /**
     *  initialize the commands from the current position, sets the
     *  gripper[EH]_cmd to where they are right now. 
     * 
     *  Return only needed because it's invokeds as a FunctionCommand
     */
    int  initialize() {
        armPosition = Robot.arm.getArmPosition();   //update position
        gripperX_cmd = armPosition.projection;
        gripperH_cmd = armPosition.height;
        xprojRL.initialize();
        xprojRL.setForward(armPosition.projection);
        heightRL.initialize(); 
        return 0;
    }

    /****************************************************************************************/
    // physical postions at this point in time as measured this frame
    double measProjection() { return armPosition.projection;  }
    double measHeight()     { return armPosition.height;     }
    /****************************************************************************************/

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
        grp.addSequential(new VacuumCommand(true));
        grp.addParallel(new MoveArmAtHeight(this::gripperHeightOut, this::gripperXProjectionOut));
        grp.addParallel(new WristTrackFunction(this::wristTrackParallel));
        return grp;
    }

    private CommandGroup CmdFactoryHuntCargo() {
        CommandGroup grp = new CommandGroup("HuntCargo");
        grp.addSequential(new VacuumCommand(true));
        grp.addParallel(new MoveArmAtHeight(this::gripperHeightOut, this::gripperXProjectionOut));
        grp.addParallel(new WristTrackFunction(this::wristTrackPerp));
        return grp;
    }

    private CommandGroup CmdFactoryHuntHatchFloor() {
        CommandGroup grp = new CommandGroup("HuntHatchFloor");
        grp.addSequential(new VacuumCommand(true));
        grp.addParallel(new MoveArmAtHeight(this::gripperHeightOut, this::gripperXProjectionOut));
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
        grp.addSequential(new RotateWristCommand(95.0, 1.0));              //these will wait, not timeout, 
        grp.addSequential(new RotateWristCommand(90.0, 1.0));              // wiggle wrist to grab hatch
        grp.addSequential(new ExtendArmToPositionCommand(5.0));            //pull in a bit before rotate, should have hatch
        grp.addSequential(new TestRotateArmToAngleCommand(150.0, 30.0));   //rotate clear before going to delivery mode

        grp.addSequential(new NextModeCmd(Modes.DeliverHatch));
        return grp;
    }

    private CommandGroup CmdFactoryCapture() {
        CommandGroup grp = new CommandGroup("Capture");
        grp.addSequential(new VacuumCommand(true));                 
        //grp.addSequential(new MoveDownToCapture(Capture_dDown), 3.5 );  //TODO: fix  3.5 seconds const
        grp.addSequential(new CallFunctionCmd(this::gotoDeliverMode));
        return grp;
    }

    private CommandGroup CmdFactoryDelivery() {
        CommandGroup grp = new CommandGroup("Deliver");
        grp.addParallel(new MoveArmAtHeight(this::gripperHeightOut, this::gripperXProjectionOut)); //use deliver gripper funct
        grp.addParallel(new WristTrackFunction(this::wristTrackParallel));
        return grp;
    }

    private CommandGroup CmdFactoryRelease() {
        CommandGroup grp = new CommandGroup("Release");
        //grp.AddSequential(new Extend_Drive_To_Deliver());
        grp.addSequential(new VacuumCommand(false));
        grp.addSequential(new WaitCommand(1.0));                  //maybe move the wrist??
        grp.addSequential(new NextModeCmd(Modes.HuntingHatch));    // go back to hunting
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

    /**
     * Very complex command that does nothing, but waits for it.
     * Useful to ensure a command group waits a period before finishing.
     */
    class WaitCommand extends Command {
        double timeout;
        WaitCommand(double timeout) {
            this.timeout = timeout;
        }
        @Override
        protected void initialize()  { setTimeout(timeout);  }
        @Override
        protected boolean isFinished() {return isTimedOut();    }
        @Override
        protected void execute() { /*nothing */ }
    }

    // Turn any function into an instant command. Return value not really used.
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
            SmartDashboard.putString("Command Mode", currentMode.toString());
            SmartDashboard.putNumber("GripHCmd", gripperH_cmd);
            SmartDashboard.putNumber("GripX_RL", heightRL.get() );
            SmartDashboard.putNumber("GripXCmd", gripperX_cmd);
            SmartDashboard.putNumber("GripX_RL", xprojRL.get() );
        }
    }
}
