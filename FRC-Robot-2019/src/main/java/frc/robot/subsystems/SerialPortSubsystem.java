package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import edu.wpi.first.hal.util.UncleanStatusException;

/**
 * What this subsystem does
 */
public class SerialPortSubsystem extends Subsystem {

//stuff it needs

private int distance1;
private int distance2;
private String serialResults;
private SerialPort arduinoSerial;
private long distanceRefresh;
private long hertz;


  public SerialPortSubsystem() {
    arduinoSerial = new SerialPort(115200, SerialPort.Port.kUSB);
  }

  @Override
  public void initDefaultCommand() {
      //set default command
  }

  public int getDistance(int sensor) {
    if (sensor==1) return distance1;
    if (sensor==2) return distance2;
    return 0;
  }

  public long getHertz() {
    return hertz;
  }


  public void processSerial() {
    String results = "";
    int sensor = 0;
    int distance = 0;
    
    //reduce buffer size to last 1000 bytes to prevent loop time overrun
    while (arduinoSerial.getBytesReceived()>1000) results = arduinoSerial.readString(); 
  
    //read buffer if available one char at a time
    while (arduinoSerial.getBytesReceived()>0) {
      try {
        results = arduinoSerial.readString(1);
      } catch (UncleanStatusException e) {
        System.out.println("Serial Exception UncleanStatusException caught. Code:" + e.getStatus());
        arduinoSerial.reset();
        
      }
      

        //E is end of statement, otherwise add to running string
        if (!results.contentEquals("E")) {
        serialResults = serialResults + results;
        }
        else {
          //Correct statement is minimum of 3 char long
          if(serialResults.length()<3) {
            serialResults="";
            System.out.println("Bad serial packet length, tossing.");
          }
          //Correct statement starts with S
          else if(serialResults.charAt(0) != 'S') {
            serialResults="";
            System.out.println("Bad serial packet start char, tossing.");
          }
          else {
              
            sensor = Character.getNumericValue(serialResults.charAt(1));
          
            if(serialResults.length()>2) {
              distance = Integer.parseInt(serialResults.substring(2));

              if(sensor==1) distance1 = distance;
              if(sensor==2) distance2 = distance;
            }
          
            serialResults="";
            distance = distance1-distance2;   
            
            Long refreshTime = System.currentTimeMillis() - distanceRefresh;
            distanceRefresh = System.currentTimeMillis();
            
            hertz=0;
            if (refreshTime>0) hertz = 1000/refreshTime;
                
            
            

            //System.out.println("Sensor#1: " + distance1 + "mm, Sensor#2: " + distance2 + "mm, Diff=" + distance + "mm, Refresh:" + refresh + "Hz");
          }
        }
      }
    }


}

