import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class TalonRobot extends TimedRobot {
    /* Hardware */
    TalonSRX _talon = new TalonSRX(1);
    Joystick _joy = new Joystick(0);

    /* String for output */
    StringBuilder _sb = new StringBuilder();

    /* Loop tracker for prints */
    int _loops = 0;

    public void robotInit() {
        /* Factory Default all hardware to prevent unexpected behaviour */
        _talon.configFactoryDefault();

        /* Config sensor used for Primary PID [Velocity] */
        _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        /**
         * Phase sensor accordingly. Positive Sensor Reading should match Green
         * (blinking) Leds on Talon
         */
        _talon.setSensorPhase(true);

        /* Config the peak and nominal outputs */
        _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
        _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
        _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        _talon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
        _talon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
        _talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
        _talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        /* Get gamepad axis */
        double leftYstick = -1 * _joy.getY();

        /* Get Talon/Victor's current output percentage */
        double motorOutput = _talon.getMotorOutputPercent();

        /* Prepare line to print */
        _sb.append("\tout:");
        /* Cast to int to remove decimal places */
        _sb.append((int) (motorOutput * 100));
        _sb.append("%"); // Percent

        _sb.append("\tspd:");
        _sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
        _sb.append("u"); // Native units

        /**
         * When button 1 is held, start and run Velocity Closed loop. Velocity Closed
         * Loop is controlled by joystick position x500 RPM, [-500, 500] RPM
         */
        if (_joy.getRawButton(1)) {
            /* Velocity Closed Loop */

            /**
             * Convert 500 RPM to units / 100ms. 4096 Units/Rev * 500 RPM / 600 100ms/min in
             * either direction: velocity setpoint is in units/100ms
             */
            double targetVelocity_UnitsPer100ms = leftYstick * 500.0 * 4096 / 600;
            /* 500 RPM in either direction */
            _talon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

            /* Append more signals to print when in speed mode. */
            _sb.append("\terr:");
            _sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
            _sb.append("\ttrg:");
            _sb.append(targetVelocity_UnitsPer100ms);
        } else {
            /* Percent Output */

            _talon.set(ControlMode.PercentOutput, leftYstick);
        }

        /* Print built string every 10 loops */
        if (++_loops >= 10) {
            _loops = 0;
            System.out.println(_sb.toString());
        }
        /* Reset built string */
        _sb.setLength(0);
    }
}