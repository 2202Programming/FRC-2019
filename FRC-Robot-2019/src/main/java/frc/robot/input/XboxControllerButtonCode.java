package frc.robot.input;

/**
 * Class for a bunch of WPI button constants
 */
public enum XboxControllerButtonCode {
    A(1),
	B(2),
	X(3),
	Y(4),
	LB(5),
	RB(6),
	BACK(7),
	START(8),
	L3(9),
	R3(10),

	TRIGGER_LEFT(2),// for left trigger and
	TRIGGER_RIGHT(3),// right trigger

	RIGHT_X(4),// for joysticks
	RIGHT_Y(5),
	LEFT_X(0),
	LEFT_Y(1);

	public final int value;
	private XboxControllerButtonCode(int initValue) {
		value = initValue;
	}

	public int getCode() {
		return value;
	}
}
