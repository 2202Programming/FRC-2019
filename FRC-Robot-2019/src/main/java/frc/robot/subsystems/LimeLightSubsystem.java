package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLightSubsystem extends Subsystem {

    private double x;
    private double y;
    private double area;
    private double target;
    private NetworkTable table;
    private long logTimer;
    
    public LimeLightSubsystem() {
        logTimer = System.currentTimeMillis();
    }

    @Override
    protected void initDefaultCommand() {

    }

    public void log(int interval) {

        if ((logTimer + interval) < System.currentTimeMillis()) { //only post to smartdashboard every interval ms
          logTimer = System.currentTimeMillis();

          SmartDashboard.putNumber("LimelightX", getX());
          SmartDashboard.putNumber("LimelightY", getY());
          SmartDashboard.putNumber("LimelightArea", getArea());
          SmartDashboard.putBoolean("LimeTarget", hasTarget());

        }
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
        if(Robot.driveTrain.getInversionConstant()>0) cameraSelect.setDouble(0);
        else cameraSelect.setDouble(1);

        return;
    }

    public void disableLED() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void enableLED() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
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