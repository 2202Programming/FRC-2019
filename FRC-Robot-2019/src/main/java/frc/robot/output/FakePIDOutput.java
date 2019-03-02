package frc.robot.output;

import edu.wpi.first.wpilibj.PIDOutput;

public class FakePIDOutput implements PIDOutput{

	//A fake motor to get PID values 
	public FakePIDOutput() {
		
	}
	@Override
	public void pidWrite(double output) {
		return;
	}
	
}
