package frc.robot.input.triggers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class MotorOverPowerShutdown extends Trigger  {
    final int TAP_MAX = 25;   // this is 0.5 seconds normally
    WPI_TalonSRX motor;
    double powerLimit;
    double avgPower;          //watts
    Command saveMotorCmd;
    LinearFilter movingWindow;

    public MotorOverPowerShutdown(WPI_TalonSRX motor, double powerLimit, double seconds) {
        this.powerLimit = powerLimit;
        this.motor = motor;    
        this.avgPower = 0.0;
        int taps = (int) Math.floor(seconds / Robot.dT);
    
        // build a moving average window
        movingWindow = LinearFilter.movingAverage(taps);
        this.saveMotorCmd = new SaveMotor();

        //install the command and hope it is never used
        this.whenActive(this.saveMotorCmd);

        System.out.println("OverPower " +motor.getName() + " watts= " + powerLimit + " - for testing only");
    }

    @Override
    public boolean get() {
        // this is called each frame, so call pidGet() here.
        // Not really a pid, but this is how the filter class works

        double power = movingWindow.calculate(readPower());
        // look for too much average power 
        if (power >= powerLimit) return true;
        return false;
    }

    // monitor power for the averaging filter
    double readPower() {
        double oi = motor.getSupplyCurrent();
        double ov = motor.getMotorOutputVoltage();
        return Math.abs(oi*ov);
    }


    // SaveMotor will disable the motor and set speed to zero of the overpower triggers
    class SaveMotor extends CommandBase {
        SaveMotor() {           
        }

        @Override
        public void execute() {
            motor.set(0.0);
            motor.disable();

            //Make noise
            System.out.println("****MOTOR POWER TRIGGERED**** -->" + motor.getName() );
        }
        
        // keep in the safe state, this command will have to get kicked out.
        @Override
        public boolean isFinished() { 
            return false; 
        }
    }

}