package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

public class LimeLightSubsystem extends Subsystem {

    private double x;
    private double y;
    private double area;
    private double target;
    private NetworkTable table;
    
    @Override
    protected void initDefaultCommand() {

    }

    public void populateLimelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");
        NetworkTableEntry cameraSelect = table.getEntry("cameraSelect");

        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        target = tv.getDouble(0.0);

        //put cameraselect value into network tables
        if(Robot.driveTrain.getInversionConstant()>0) cameraSelect.setBoolean(true);
        else cameraSelect.setBoolean(false);

        return;
    }

    public NetworkTable getNetworkTable() {
        return table;
    }

    public double getX() {
        return x;
    }

    public double getY() {
         return y;
    }

    public double getArea() {
        return area;
    }

    public Boolean hasTarget() {
        if (target < 1) return false;
        else return true;
    }

}