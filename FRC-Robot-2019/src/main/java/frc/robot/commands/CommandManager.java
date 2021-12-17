package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmStatePositioner;
import frc.robot.commands.arm.MoveArmToRawPosition;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.intake.RetractOnReleaseCommand;
import frc.robot.commands.intake.VacuumCommand;
import frc.robot.commands.intake.WristSetAngleCommand;
import frc.robot.commands.intake.WristTrackAngle;
import frc.robot.commands.util.Angle;
import frc.robot.commands.util.MathUtil;
import frc.robot.commands.util.TriggerTimeoutCommand;
import frc.robot.subsystems.VacuumSensorSystem;

/**
 * One class, singleton, to rule them all. Coordinates major modes of operation.
 * Handles tracking which commands get set when.
 * 
 */
public class CommandManager {
    public final double kCapHeight = 4.0; // inch/joy units TODO: put in better place
    public final double kHeightMin = 2.0; // inches
    public final double kHeightMax = 96.0; // inches

    int logCnt = 0;
    private long logTimer;

    // Button Commands
    Command huntSelectCmd;
    Command heightSelectCmd;
    Command captRelCmd;
    Command flipCmd;

    // Command Sets - one created for each major operation of the robot.
    Command zeroRobotGrp;
    Command huntGameStartGrp;
    Command huntingHatchGrp;
    Command huntingCargoGrp;
    Command huntingHFloorGrp;
    Command captureGrp;
    Command deliveryGrp;
    Command releaseGrp;
    Command flipToFrontGrp;
    Command flipToFrontGrpFast;
    Command flipToBackGrp;
    Command flipToBackGrpFast;
    Command driveGrp;
    Command currentGrp; // what is running

    // Modes of behavior
    public enum Modes {
        Construction(0), // system still coming up... not operational
        SettingZeros(1), // calling all encoder power on requirements, not operational
        // Operational modes
        HuntGameStart(2), // special hunting piece mode - hunt the one on our robot
        HuntingHatch(3), // Button:HuntSelect order 3->4->5--3-->4-->5...
        HuntingCargo(4), // HuntSelect
        HuntingFloor(5), // HuntSelect
        // Capture
        Capturing(6), // TODO: UNUSED right now,
        // TODO: Make sure capturing leads right to recapturing, and make recapturing
        // lead to deliver (if not dropped), hunting/capturing otherwise
        Recapturing(7), // UNUSED right now 3-second period where robot will be able to recapture
                        // hatch/cargo if dropped
        // DeliveryModes
        Drive(8), Defense(9), // Unused, for when we need to go to the other side
        DeliverHatch(10), // based on what we captured
        DeliverCargo(11), // based on what we captured
        Flipping(12), Releasing(20), // Button:CaptureRelease
        Climbing(21);

        private int v;

        Modes(int v) {
            this.v = v;
        }

        public int get() {
            return v;
        }
    }

    // Target States - think of this as desired command vector
    Modes currentMode; // what we think are doing now
    Modes prevHuntMode = Modes.HuntingHatch; // what we caught, used to tell if hatch or cargo
    Modes prevMode;

    // internal states
    private final Modes huntingModes[] = { Modes.HuntingFloor, Modes.HuntingCargo, Modes.HuntingHatch };
    private int huntModeIdx = 2; // hatch
    private int delHeightIdx = 0; // used in Delivery<Cargo/Hatch>Heights[]
    private int driveIdx = 1;

    public CommandManager() {
        currentMode = Modes.Construction;
        // bind commands to buttons
        Robot.m_oi.heightDownSelect.whenPressed(new CallFunctionCmd(this::cycleDown));
        Robot.m_oi.heightUpSelect.whenPressed(new CallFunctionCmd(this::cycleUp));
        Robot.m_oi.captureRelease.whenPressed(new CallFunctionCmd(this::triggerCaptureRelease));
        Robot.m_oi.flip.whenPressed(new FlipCmd());
        Robot.m_oi.endDriveMode.whenPressed(new CallFunctionCmd(this::endDriveState));
        Robot.m_oi.goToPrevMode.whenPressed(new CallFunctionCmd(this::goToPrevMode));

        // Rumble on vacuum
        VacuumSensorSystem vs = Robot.intake.getVacuumSensor();
        if ((vs != null) && vs.isGood()) {
            // Command vacRumble = new RumbleCommand(Robot.m_oi.getAssistantController(), vs::hasVacuum);
            // capture trigger on vacuum
            Trigger capTrigger = new Trigger() {
                public boolean get() {
                    return vs.hasVacuum();
                }
            };
            capTrigger.whenActive(new CallFunctionCmd(this::autoTriggerCapture));
        }

        logTimer = System.currentTimeMillis();

        // Construct our major modes from their command factories
        zeroRobotGrp = CmdFactoryZeroRobot();
        huntGameStartGrp = CmdFactoryHuntGameStart();
        huntingHFloorGrp = CmdFactoryHuntHatchFloor();
        huntingHatchGrp = CmdFactoryHuntHatch();
        huntingCargoGrp = CmdFactoryHuntCargo();
        captureGrp = CmdFactoryCapture();
        driveGrp = CmdFactoryDrive();
        deliveryGrp = CmdFactoryDelivery();
        releaseGrp = CmdFactoryRelease();
        flipToFrontGrp = CmdFactoryFlipToFront(); // (dpl - keep from mistakes for now) CmdFactoryFlip();
        flipToFrontGrpFast = CmdFactoryFlipToFrontFast(); // (dpl - keep from mistakes for now) CmdFactoryFlip();
        flipToBackGrp = CmdFactoryFlipToBack();
        flipToBackGrpFast = CmdFactoryFlipToBackFast();
    }

    /**
     * Handle the state transitions, set any state vars and setup next command
     * group.
     *
     * Gripper height setting is controlled by state changes while hunting so it set
     * in the here in setMode(). It uses the gripperHeight() to deliver the value to
     * the running commands.
     * 
     * When in delivery mode, the height is calculated by cycling through an array
     * of heights. The array index is bumpped on heightSelect button. The value is
     * delivered to the commands via deliverGripperHeight();
     * 
     * Different MoveArmAtHeight commands instantiated with the different
     * gripperHeight functions assigned.
     */
    public void setMode(Modes mode) {
        Command nextCmd = null;

        switch (mode) {
        case Construction:
            break;

        case SettingZeros:
            nextCmd = zeroRobotGrp;
            break;

        case HuntGameStart:
            prevHuntMode = Modes.HuntingHatch; // change this if we start with Cargo
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

        case Capturing: // moving from hunting to picking it up. Button:Capture
            prevHuntMode = currentMode; // this is what we captured
            nextCmd = captureGrp;
            break;
        case Drive:
            driveIdx = 0;
            nextCmd = driveGrp;
            break;
        // DeliveryModes
        case DeliverHatch: // based on what we captured
            delHeightIdx = 1; // start at lowest
            nextCmd = deliveryGrp;
            break;

        case DeliverCargo: // based on what we captured
            delHeightIdx = 1;
            nextCmd = deliveryGrp;
            break;

        case Releasing:
            prevHuntMode = Modes.Releasing; // Reset the prevHuntMode
            nextCmd = releaseGrp;
            break;
        case Flipping:
            if (Robot.arm.isInverted()) {
                if (Robot.intake.getVacuumSensor().hasVacuum()) {
                    nextCmd = flipToFrontGrp;
                } else {
                    nextCmd = flipToFrontGrpFast;
                }
            } else {
                if (Robot.intake.getVacuumSensor().hasVacuum()) {
                    nextCmd = flipToBackGrp;
                } else {
                    nextCmd = flipToBackGrpFast;
                }
            }
            break;
        default:
            break;
        }

        // update states and start command group
        prevMode = currentMode;
        currentMode = mode;
        // calculate the height and extension, set gripperH_cmd, and gripperE_cmd
        installGroup(nextCmd);
    }

    // Starts new group.
    void installGroup(Command grp) {
        if (grp == null)
            return;
        if (currentGrp != null)
            currentGrp.cancel(); // end methods are called
        currentGrp = grp;
        currentGrp.schedule();   // schedule our new work, initialize() then execute() are called
    }

    public Modes getCurMode() {
        return currentMode;
    }

    public int getPositionIndex() {
        switch (getCurMode()) {
        case Drive:
            return driveIdx;
        case DeliverHatch:
            return delHeightIdx;
        case DeliverCargo:
            return delHeightIdx;
        default:
            return 0;
        }
    }

    public boolean isHunting() {
        if ((currentMode.get() > Modes.HuntGameStart.get()) && (currentMode.get() < Modes.Capturing.get())) {
            return true;
        }
        return false;
    }

    public boolean isDelivering() {
        return ((currentMode == Modes.DeliverCargo) || (currentMode == Modes.DeliverHatch));
    }

    public boolean isDriving() {
        return ((currentMode == Modes.Drive) || (currentMode == Modes.Defense));
    }

    // called on capture/release trigger button
    private int triggerCaptureRelease() {
        if (isHunting()) {
            prevHuntMode = currentMode;
            setMode(Modes.Drive);
        } else
            setMode(Modes.Releasing);
        return 0;
    }

    /**
     * Similar to triggerCaptureRelease, but floor handled differently. When we get
     * vacuum we goto drive mode. When on the floor we don't want to just jump to
     * drive because the hatch may be in awkward spot. Let the driver know by moving
     * up a few inches.
     * 
     */
    private int autoTriggerCapture() {
        if (isHunting()) {
            prevHuntMode = currentMode;
            setMode(Modes.Drive); // got it, go to Drive
        }
        // not hunting not sure HTH we got here - do nothing
        return 0;
    }

    private int endDriveState() {
        if (isDriving()) {
            if (prevHuntMode == Modes.Releasing) {
                // If we came to driving after a delivery
                setMode(Modes.HuntingHatch);
                return 0;
            } else {
                // If we came to driving after a hunt
                gotoDeliverMode();
                return 0;
            }
        }
        return -1;
    }

    private void flip() {
        setMode(Modes.Flipping);
    }

    /**************************************************************************************************
     * Used for both hunting and delivering, called by LB/RB on assistant controller
     * 
     * @param direction 1--> go up -1 --> go down
     * @return
     */
    private int cycleHeightMode(int direction) {
        if (isHunting()) {
            int idx = huntModeIdx + direction;
            prevHuntMode = huntingModes[huntModeIdx];
            huntModeIdx = MathUtil.limit(idx, 0, huntingModes.length - 1);
            setMode(huntingModes[huntModeIdx]);
            return huntModeIdx;
        } else if (isDelivering()) {
            int idx = delHeightIdx + direction; // next height
            // make sure index fits in array
            delHeightIdx = MathUtil.limit(idx, 0, ArmStatePositioner.DeliveryCargoPositions[0].length - 1);
            return delHeightIdx;
        } else if (isDriving()) {
            int idx = driveIdx + direction; // next height
            driveIdx = MathUtil.limit(idx, 0, ArmStatePositioner.DrivePositions[0].length - 1); // make sure index fits
                                                                                                // in array

            return driveIdx;
        }
        return (-1);
        // todo: drive delivery here???
    }

    // call the above code either going up or down, used by LB or RB
    private int cycleUp() {
        return cycleHeightMode(1);
    }

    private int cycleDown() {
        return cycleHeightMode(-1);
    }

    private int goToPrevMode() {
        if (currentMode == Modes.Drive) {
            if ((prevMode.get() > Modes.HuntGameStart.get()) && (prevMode.get() < Modes.Capturing.get())) {
                setMode(prevMode);
                prevMode = Modes.Releasing; // Intentially overriding the previous to Releasing if we go back
                return prevMode.get();
            }
        }
        return 0;
    }

    /******************************************************************************************** */

    // Select proper delivery mode based on what we were hunting.
    private int gotoDeliverMode() {
        // prevHunt mode saved on entering capturing mode... use it for hatch v cargo
        // delivery
        Modes nextMode = (prevHuntMode == Modes.HuntingCargo) ? Modes.DeliverCargo : Modes.DeliverHatch;
        setMode(nextMode);
        return (nextMode.get());
    }

    /************************************************************************************************************/

    // called every frame, reads inputs, does rate limiting
    public void execute() {
    }

    /**
     * initialize the commands from the current position, sets the gripper[EH]_cmd
     * to where they are right now.
     * 
     * Return only needed because it's invokeds as a FunctionCommand
     */
    int initialize() {
        return 0;
    }

    /****************************************************************************************/

    // Command Factories that build command sets for each mode of operation
    // These are largely interruptable so we can switch as state changes
    private Command CmdFactoryZeroRobot() {
        return  new SequentialCommandGroup(
            Robot.arm.zeroSubsystem(),
            Robot.intake.zeroSubsystem(),
            new CallFunctionCmd(this::initialize)).withName("ZeroRobot");

        // commands to come
        /// grp.addParallel(Robot.cargoTrap.zeroSubsystem());
       
    }

    private Command CmdFactoryHuntHatch() {
        return  new VacuumCommand(true, 0.0).withName("HuntHatch");
       
    }

    private Command CmdFactoryHuntCargo() {
        return new SequentialCommandGroup(
            new VacuumCommand(true, 0.0)).withName("HuntCargo");
    }

    private Command CmdFactoryHuntHatchFloor() {
        return new VacuumCommand(true, 0.0).withName("HuntHatchFloor");
    }

    // Hunt Game Start will have a game piece at a starting place
    // that will require some special motion, turn on vacuum,
    // and finaly go into delivery.
    //
    private Command CmdFactoryHuntGameStart() {
        VacuumSensorSystem vs = Robot.intake.getVacuumSensor();
        
        return  new SequentialCommandGroup(
    
       new VacuumCommand(true, 0.0),
        grp.addParallel(new WristTrackAngle(Angle.Starting_Hatch_Hunt.getAngle()));
    
       new MoveArmToPosition(4.875, 11, 0.05, 1),       // Move arm up and back to avoid moving hatch
       new MoveArmToPosition(4.875, 12.75, 0.05, 1),    // Move arm into hatch and intake
       new MoveArmToPosition(4.875, 13.5, 0.05, 1),     // Move arm into hatch and intake
       new TriggerTimeoutCommand(vs::hasVacuum, 1.0),   // waits or sees vacuum and finsishes
       new MoveArmToPosition(13, 12, 0.05, 1) );
    
        addParallel(new WristTrackAngle(Angle.Parallel.getAngle()));
    
            new NextModeCmd(Modes.HuntingHatch), // Capture the right previous state
            new NextModeCmd(Modes.Drive),
            new NextModeCmd(Modes.DeliverHatch),
       ).withName("HuntGameStart");
    }

    private Command CmdFactoryCapture() {
        return  new SequentialCommandGroup(
            new VacuumCommand(true, 0.0),       // no timeout
                //new MoveDownToCapture(Capture_dDown), 3.5 ); //TODO: fix
                // 3.5 seconds const
            new CallFunctionCmd(this::gotoDeliverMode)   ).withName("Capture");
    }

    private Command CmdFactoryDrive() {
        return new SequentialCommandGroup().withName("Driving");
    }

    private Command CmdFactoryDelivery() {
        return new SequentialCommandGroup().withName("Deliver");
    }

    private Command CmdFactoryRelease() {
        double vacTimeout = 0.2; // seconds
        return new SequentialCommandGroup(
            //new Extend_Drive_To_Deliver(),   //dpl tbd 12/17/21
            new VacuumCommand(false, vacTimeout),
            new RetractOnReleaseCommand(this, 4.0 /* inchs */, 1.0),
            new NextModeCmd(Modes.Drive) // go back to driving configuration
        ).withName("Release");
    }

    // TODO: Check for working w/ higher speeds
    private Command CmdFactoryFlipToBack() {
        return new SequentialCommandGroup(
            new WristSetAngleCommand(0.0),
            new MoveArmToRawPosition(-35.0, 12.0, 1.0, 180),
            new PrevCmd() ).withName("FlipToBack");
    }

    // TODO: Check for working w/ higher speeds
    private Command CmdFactoryFlipToBackFast() {
        return new SequentialCommandGroup(
            new WristSetAngleCommand(0.0),
            new MoveArmToRawPosition(-35.0, 12.0, 1.0, 360),
            new PrevCmd()).withName("FlipToBackFast");
    }

    // TODO: Check for working w/ higher speeds
    private Command CmdFactoryFlipToFront() {
        return  new SequentialCommandGroup(
            new WristSetAngleCommand(0.0),
            new MoveArmToRawPosition(35.0, 12.0, 1.0, 180),
            new PrevCmd() ).withName("FlipToFront");
    }

    // TODO: Check for working w/ higher speeds
    private Command CmdFactoryFlipToFrontFast() {
     return new SequentialCommandGroup(
                new WristSetAngleCommand(0.0),
                new MoveArmToRawPosition(35.0, 12.0, 1.0, 360),
                new PrevCmd() ).withName("FlipToFrontFast");
    }

    // Used at the end of a command group to jump to next mode
    class NextModeCmd extends InstantCommand {
        Modes mode2set;

        NextModeCmd(Modes m) {
            mode2set = m;
        }

        @Override
        public void execute() {
            setMode(mode2set);
        }
    }

    class FlipCmd extends InstantCommand {
        @Override
        public void execute() {
            flip();
        }
    }

    class PrevCmd extends InstantCommand {
        @Override
        public void execute() {
            setMode(prevMode);
        }
    }

    // Turn any function into an instant command. Return value not really used.
    public class CallFunctionCmd extends InstantCommand {
        IntSupplier workFunct;

        public CallFunctionCmd(IntSupplier workFunct) {
            this.workFunct = workFunct;
        }

        @Override
        public void execute() {
            workFunct.getAsInt();
        }
    }

    public void log(int interval) {
        if ((logTimer + interval) < System.currentTimeMillis()) { // only post to smartdashboard every interval ms
            logTimer = System.currentTimeMillis();
            SmartDashboard.putString("Command Mode", currentMode.toString());
        }
        SmartDashboard.putString("Current Position", logCurHeight());
    }

    public String logCurHeight() {
        String position = "";
        String[] delivery = { "Low", "Middle", "High" };
        switch (currentMode) {
        case Construction:
            position = "Constructing";
            break;
        case SettingZeros:
            position = "Setting Zeros";
            break;
        case HuntGameStart:
            position = "Starting up";
            break;
        case HuntingHatch:
            position = "Hunting hatch";
            break;
        case HuntingCargo:
            position = "Hunting cargo";
            break;
        case HuntingFloor:
            position = "Hunting floor hatch";
            break;
        case Drive:
            position = "Driving";
            break;
        case Defense:
            position = "Defending";
            break;
        case DeliverHatch:
            position = "Delivering Hatch " + delivery[delHeightIdx];
            break;
        case DeliverCargo:
            position = "Delivering Cargo " + delivery[delHeightIdx];
            break;
        case Flipping:
            position = "Flipping";
            break;
        case Releasing:
            position = "Releasing";
            break;
        default:
            position = "???";
            break;
        }
        return position;
    }
}