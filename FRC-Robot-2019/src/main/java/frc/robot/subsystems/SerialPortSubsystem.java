package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.hal.util.UncleanStatusException;
import java.lang.StringBuilder;

//Listens on USB Serial port for LIDAR distance data from the arduino

public class SerialPortSubsystem extends Subsystem {

private int distance1; //Sensor #1
private int distance2; //Sensor #2
private StringBuilder serialResults;
private SerialPort arduinoSerial;
private long distanceRefresh; //Track time between sensor readings
private long hertz;
private boolean serialExists = true;

  public SerialPortSubsystem() {
    try {
      arduinoSerial = new SerialPort(115200, SerialPort.Port.kUSB);
    }
    catch(UncleanStatusException e) {
      serialExists = false;
    }

  }

  public boolean isSerialEnabled() {
    return serialExists;
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
    byte[] results;
    int sensor = 0;
    int distance = 0;
    
    //reduce buffer size to last 1000 bytes to prevent loop time overrun
    while (arduinoSerial.getBytesReceived()>1000) {
      results = arduinoSerial.read(1);
    }
    //read buffer if available one char at a time
    while (arduinoSerial.getBytesReceived()>0) {
      try {
        results = arduinoSerial.read(1);  //dpl was  .readString(1);
      } catch (UncleanStatusException e) {     //Catch uncleanstatusexception and restart serial port 
        System.out.println("Serial Exception UncleanStatusException caught. Code:" + e.getStatus());
        arduinoSerial.reset();
        return;
      }
        //FORMAT is S[# of sensor, 1-4][Distance in mm]E
        //E is end of statement, otherwise add to running string
        if (!results.contentEquals("E")) {
        serialResults.append(results);
        }
        else {
          //Correct statement is minimum of 3 char long
          if(serialResults.length()<3) {
            serialResults.delete(0, serialResults.length());
            System.out.println("Bad serial packet length, tossing.");
          }
          //Correct statement starts with S
          else if(serialResults.charAt(0) != 'S') {
            serialResults.delete(0, serialResults.length());
            System.out.println("Bad serial packet start char, tossing.");
          }
          else {
              
            sensor = Character.getNumericValue(serialResults.charAt(1));
          
            if(serialResults.length()>2) {
              distance = Integer.parseInt(serialResults.substring(2));

              if(sensor==1) distance1 = distance;
              if(sensor==2) distance2 = distance;
            }
          
            serialResults.delete(0, serialResults.length());
                       
            Long refreshTime = System.currentTimeMillis() - distanceRefresh;
            distanceRefresh = System.currentTimeMillis();
            
            hertz=0;
            if (refreshTime>0) hertz = 1000/refreshTime;
          }
        }
      }
    }


}

