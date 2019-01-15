package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A class that will read from Xbox Controller inputs and make the Xbox
 * controller rumble. I will do this eventually --SecondThread
 */
public class XboxController extends Joystick{
    private final int RIGHT_JOYSTICK_X_AXIS = 4;// for joysticks
	private final int RIGHT_JOYSTICK_Y_AXIS = 5;
	private final int LEFT_JOYSTICK_X_AXIS = 0;
    private final int LEFT_JOYSTICK_Y_AXIS = 1;
    
    public XboxController(int port) {
        super(port);
    }

    public double getRightJoystickX() {
        return getRawAxis(RIGHT_JOYSTICK_X_AXIS);
    }

    public double getRightJoystickY() {
        return getRawAxis(RIGHT_JOYSTICK_Y_AXIS);
    }

    public double getLeftJoystickX() {
        return getRawAxis(LEFT_JOYSTICK_X_AXIS);
    }

    public double getLeftJoystickY() {
        return getRawAxis(LEFT_JOYSTICK_Y_AXIS);
    }


}
