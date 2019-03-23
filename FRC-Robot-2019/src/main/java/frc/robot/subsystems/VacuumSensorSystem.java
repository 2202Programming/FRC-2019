package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.AnalogInput;
/**
 * VacuumSensor sub-system owned by the intake system
 * 
 * Uses an MPX550DP 500kPa differential pressure sensor to detect when we have a 
 * positive seal on the payload.
 * 
 * Low pass filter done in hardware to clean up. 
 * 
 *   DC bias = 0.2 V  if we don't see this it indicates a sensor failure
 * 
 *   DC Max = 5v  indicates a max pressure of 72.3 psi - we won't see that.
 * 
 *   We expect to see .4 to .8 volts with the payload. Suction sensor on bench testing 
 *   showed a 1st order response from .2 to .8v over 1.0 seconds for steady state.  
 *   This puts the time constant at .2 Sec.  Anything over .4v should be a solid capture.
 * 
 */

 public class VacuumSensorSystem extends ExtendedSubSystem {
    //physical units
    final double vacuumTriggerV = 0.32; // volts about 8psi 
    final double vacuumBiasV = 0.15;    // part or wiring is bad, 25% lower than expected .2v
    final double vacuumReleaseV = 0.22;  //point where we say we let the payload go - near bias v

    // a/d is 12 bits + extra bits added via over sample, 
	// so 12 + 3 ==> 15 bits 0 - 32767 range on averageValue
    // 12 + 1 ==> 13 bits 0 - 8191
    final int kOverSample  = 4;                             // bits accuracy 
    final int HACK = 4;     // SCALING WAS OFF - DONT KNOW WHY - DPL
    final int kMax = (int)Math.pow(2, 12 + kOverSample) / HACK;
    final int kBitAvg = 4;                                  // 2^bit = 16 samples
    
    // convert physical units to scaled ints for tests
    final int vacTriggerC = (int)(vacuumTriggerV * kMax);
    final int vacBiasC = (int)(vacuumBiasV * kMax); 
    final int vacRelease = (int)(vacuumReleaseV * kMax);   //TODO: need to test value and airpuff effect

    //Hz to run the A/D on. sampRate / 16 samples ==> 3906.25hz
    final int kSampleRate = 62500;  

    private AnalogInput sensor;
    private boolean sensorGood = false;

    public VacuumSensorSystem(int ADchan)
    {
        super("vacSensor"+ADchan);
        AnalogInput.setGlobalSampleRate(62500);   //warning this could affect others if other a/d used
        //setup sensor and test for it's goodness
		sensor = new AnalogInput(ADchan);		
		sensor.setOversampleBits(kOverSample);
		sensor.setAverageBits(kBitAvg);   
        testBias();
        // make some noise if sensor is bad.
        if (isGood() == false) {
            System.out.println(this.getName() + " failed testing at construction. ave = " + sensor.getAverageValue() );
        }
        sensorGood = true;  //hack
	}

    // looks for expected bias, used to know if we can create a trigger for the event.
    private void testBias() {
        try {
            int ave1 = sensor.getAverageValue();
            Thread.sleep(20);
            int ave2 = sensor.getAverageValue(); 
            // vacBiasC is >> 0 so if we are 
            sensorGood =  ((ave1 > vacBiasC) || (ave2 > vacBiasC)) ? true : false;
        } catch (InterruptedException e) {
            sensorGood = false;   //don't know what could have happened, call it bad
        }
    }

    public boolean hasVacuum() {
        int ave = sensor.getAverageValue();
        boolean retval = (ave > vacTriggerC) ? true : false;
        return retval;
    }

    // on release, the vac pressure should be down around the bias value.
    public boolean hasReleased() {
        int ave = sensor.getAverageValue();
        boolean retval = (ave <= vacRelease) ? true : false;
        return retval;
    }

    public int getRawVacuum() {
        return sensor.getAverageValue();
    }

    public boolean isGood() {return sensorGood; }


    @Override
    protected void initDefaultCommand() {
        // could put the trigger here if good, but may just want to do in state manager or owning subsytem
    
    }

    @Override
    public Command zeroSubsystem() {   
        return null;    //does not need a zeroSubsystem
    }
}