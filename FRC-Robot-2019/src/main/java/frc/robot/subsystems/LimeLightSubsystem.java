package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;
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

    private Deque<Double> xArray = new ArrayDeque<Double>();
    private Deque<Double> yArray = new ArrayDeque<Double>();
    private Deque<Double> areaArray = new ArrayDeque<Double>();
    private Deque<Double> targetArray = new ArrayDeque<Double>();

    
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

          SmartDashboard.putNumber("Avg Limelight X", getXAvg());
          SmartDashboard.putNumber("Avg Limelight Y", getYAvg());
          SmartDashboard.putNumber("Avg Limelight Area", getAreaAvg());
          SmartDashboard.putBoolean("Target Reliability", hasTargetReliable());



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

        xArray.add(x);//add to rolling average array
        if(xArray.size() > 10)//keep most recent 10 valuse in array
        {
          xArray.remove(); //remove oldest reading (queue)
        }

        yArray.add(y);//add to rolling average array
        if(yArray.size() > 10)//keep most recent 10 valuse in array
        {
          yArray.remove(); //remove oldest reading (queue)
        }

        areaArray.add(area);//add to rolling average array
        if(areaArray.size() > 10)//keep most recent 10 valuse in array
        {
          areaArray.remove(); //remove oldest reading (queue)
        }

        targetArray.add(target);//add to rolling average array
        if(targetArray.size() > 10)//keep most recent 10 valuse in array
        {
          targetArray.remove(); //remove oldest reading (queue)
        }


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

    public double getXAvg() { //gets olympic avg of x coordinate lydar
        double average = 0.0;
        double sum = 0.0;
        double max = 0.0;
        double min = 10000.0;
        Double[] tempArray = xArray.toArray(new Double[0]);
    
        for (int i = 0; i< xArray.size(); i++) {//gets min and max (so it can remove min and max)
          if(tempArray[i] > max)
          {
            max = tempArray[i];
          }
          if(tempArray[i] < min)
          {
            min = tempArray[i];
          }
          sum = sum + tempArray[i];
        }
        if ((xArray.size()-2)!=0) { //make sure not to divide by zero if array is small
            average = (sum-max-min) / (xArray.size()-2); //return average, not including max or min reading (olympic)  
            return average; 
        }
        else return 0;

        
    }

    public double getY() {
         return y;
    }

    public double getYAvg() {//gets olympic avg of y coordinate lydar
        double average = 0;
        double sum = 0;
        double max = 0;
        double min = 10000;
        Double[] tempArray = yArray.toArray(new Double[0]);
    
        for (int i = 0; i< yArray.size(); i++) {
          if(tempArray[i] > max)
          {
            max = tempArray[i];
          }
          if(tempArray[i] < min)
          {
            min = tempArray[i];
          }
          sum = sum + tempArray[i];
        }

        if ((yArray.size()-2)!=0) { //make sure not to divide by zero if array is small
            average = (sum-max-min) / (yArray.size()-2); //return average, not including max or min reading (olympic)  
            return average; 
            }
            else return 0;        
    }

    public double getArea() {
        return area;
    }

    public double getAreaAvg() {//get area olypmic average
        double average = 0;
        double sum = 0;
        double max = 0;
        double min = 10000;
        Double[] tempArray = areaArray.toArray(new Double[0]);
    
        for (int i = 0; i< areaArray.size(); i++) {
          if(tempArray[i] > max)
          {
            max = tempArray[i];
          }
          if(tempArray[i] < min)
          {
            min = tempArray[i];
          }
          sum = sum + tempArray[i];
        }
        if ((areaArray.size()-2)!=0) { //make sure not to divide by zero if array is small
            average = (sum-max-min) / (areaArray.size()-2); //return average, not including max or min reading (olympic)  
            return average; 
        }
        else return 0;
    } 

    public Boolean hasTarget() {
        if (target < 1) return false;
        else return true;
    }

    public Boolean hasTargetReliable() { //determines how reliable target is; if any of the values are 0 it isn't reliable
        Double[] tempArray = targetArray.toArray(new Double[0]);

        for (int i = 0; i < targetArray.size(); i++)
        {
            if(tempArray[i] == 0)
            {
                return false;
            }
        }
        return true; 

    }

}