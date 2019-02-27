package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.hal.util.UncleanStatusException;
import java.lang.StringBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.ArrayList;

//Listens on USB Serial port for LIDAR distance data from the arduino

public class SerialPortSubsystem extends Subsystem {

private int[] distanceArray = new int[4];
private ArrayList<Deque> distanceAvgArray;
private StringBuilder serialResults =new StringBuilder();
private SerialPort arduinoSerial;
private boolean serialExists = true;
private long logTimer;

  public SerialPortSubsystem() {
    try {
      arduinoSerial = new SerialPort(115200, SerialPort.Port.kUSB);
    }
    catch(UncleanStatusException e) {
      serialExists = false; //if fails to init, stop future attempts to check serial buffer
    }

    logTimer = System.currentTimeMillis();
    distanceAvgArray = new ArrayList<Deque>();
    distanceAvgArray.add(0, new ArrayDeque<Integer>());
    distanceAvgArray.add(1, new ArrayDeque<Integer>());
    distanceAvgArray.add(2, new ArrayDeque<Integer>());
    distanceAvgArray.add(3, new ArrayDeque<Integer>());


  }

  public boolean isSerialEnabled() {
    return serialExists;
  }

  @Override
  public void initDefaultCommand() {
      //set default command
  }

  public int getDistance(int sensor) { //return last distance measure for LIDAR sensor
    return distanceArray[sensor];
  }

  public Integer getDistanceAvg(int sensor) { //return olympic average of last 10 readings of LIDAR sensor
    
    Deque<Integer> tempDeque = distanceAvgArray.get(sensor);
    Integer[] tempArray = tempDeque.toArray(new Integer[0]); //switch from ArrayDeque to actual array so we can get each element
    Integer average = 0;
    Integer sum = 0;
    Integer max = 0;
    Integer min = 10000;

    for (int i = 0; i<distanceAvgArray.get(sensor).size(); i++) {
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
    average = (sum-max-min) / (distanceAvgArray.get(sensor).size()-2); //return average, not including max or min reading (olympic)  

    return average; 
    }


  public Boolean allDigits(String tempString) {
    for (int i = 0; i<tempString.length(); i++) { //check all chars to make sure they are all digits
      if (!Character.isDigit(tempString.charAt(i)))
        return false;
    }
    return true;
  }

  public void log(int interval) {

    if ((logTimer + interval) < System.currentTimeMillis()) { //only post to smartdashboard every interval ms
      logTimer = System.currentTimeMillis();
      SmartDashboard.putBoolean("Serial Enabled?", isSerialEnabled());
      if (isSerialEnabled()) { //verify serial system was initalized before calling for results
        SmartDashboard.putNumber("Left Front LIDAR (mm)", getDistance(RobotMap.LEFT_FRONT_LIDAR));
        SmartDashboard.putNumber("Right Front LIDAR (mm)", getDistance(RobotMap.RIGHT_FRONT_LIDAR));
        SmartDashboard.putNumber("Left Back LIDAR (mm)", getDistance(RobotMap.LEFT_BACK_LIDAR));
        SmartDashboard.putNumber("Right Back LIDAR (mm)", getDistance(RobotMap.RIGHT_BACK_LIDAR));

        SmartDashboard.putNumber("Avg Left Front LIDAR (mm)", getDistanceAvg(RobotMap.LEFT_FRONT_LIDAR));
        SmartDashboard.putNumber("Avg Right Front LIDAR (mm)", getDistanceAvg(RobotMap.RIGHT_FRONT_LIDAR));
        SmartDashboard.putNumber("Avg Left Back LIDAR (mm)", getDistanceAvg(RobotMap.LEFT_BACK_LIDAR));
        SmartDashboard.putNumber("Avg Right Back LIDAR (mm)", getDistanceAvg(RobotMap.RIGHT_BACK_LIDAR));
        }
    }
  }

  public void serialReduction(int bufferLimit) { //throw away serial buffer contents until size is < bufferLimit
    String temp;

    while (arduinoSerial.getBytesReceived() > bufferLimit) {
      try {
        temp = arduinoSerial.readString(1);  //dpl was  .readString(1);
      } 
      catch (UncleanStatusException e) {     //Catch uncleanstatusexception and restart serial port 
        System.out.println("Serial Exception UncleanStatusException caught. Code:" + e.getStatus());
        arduinoSerial.reset();
        return;
      }
    }
    return;
  }

  public void serialFlush(int bufferLimit) {
    int flushAmount = arduinoSerial.getBytesReceived()-bufferLimit;
    if(flushAmount>0) {
      try{
        byte[] temp = arduinoSerial.read(flushAmount);
        System.out.println("Flushed " + (flushAmount) + " bytes from serial buffer");
      } 
      catch (UncleanStatusException e) {     //Catch uncleanstatusexception and restart serial port 
        System.out.println("Serial Exception UncleanStatusException caught. Code:" + e.getStatus());
        arduinoSerial.reset();
        return;
      }
    }
    return;
  }

  public void processSerial() {
    String results;
    
    if (serialExists) {  //only run if serial port was correctly initalized

      //reduce buffer size to last 500 bytes to prevent loop time overrun
      //serialReduction(500);
      serialFlush(500);

      //read buffer if available one char at a time
      while (arduinoSerial.getBytesReceived()>0) {
        try {
          results = arduinoSerial.readString(1);  //dpl was  .readString(1);
        } 
        catch (UncleanStatusException e) {     //Catch uncleanstatusexception and restart serial port 
          System.out.println("Serial Exception UncleanStatusException caught. Code:" + e.getStatus());
          arduinoSerial.reset();
          return;
        }
        //FORMAT is S[# of sensor, 1-4][Distance in mm]E
        //E is end of message, otherwise add to running string
        if (results.charAt(0) != 'E') {
          serialResults.append(results); //message not done, append byte
        }
        else { //Found an E, time to process message (without E at end)
          processMessage(serialResults);
          serialResults.delete(0, serialResults.length()); //message processed, delete
        }
      }
    }
  }
  

  public void processMessage(StringBuilder serialResults){
    int sensor = 0;
    int distance = 0;

    if(serialResults.length()<3) { //correct message is at least 3 char long
      System.out.println("Bad serial packet length, tossing.");
      return;
    }

    if(serialResults.charAt(0) != 'S') { //Correct statement starts with S
      System.out.println("Bad serial packet start char, tossing.");
      return;
    }
    
    if (!Character.isDigit(serialResults.charAt(1))) { // if 2nd byte is a number (it should be) this is the sensor #
      System.out.println("Bad serial packet sensor #, tossing.");
      return;
    }

    sensor = Character.getNumericValue(serialResults.charAt(1));

    if (!allDigits(serialResults.substring(2))) { //only convert to integer if everything left in the string are digits
      System.out.println("Bad serial packet distance #, tossing.");
      return;
    }

    distance = Integer.parseInt(serialResults.substring(2)); //read in everything after first two char as a distance (in mm)

    distanceArray[sensor]=distance;//set read distance to correct LIDAR based on read sensor ID
    distanceAvgArray.get(sensor).add(distance);//add to rolling average array
    if(distanceAvgArray.get(sensor).size() > 10)//keep most recent 10 valuse in array
    {
      distanceAvgArray.get(sensor).remove();
    }
  }
}