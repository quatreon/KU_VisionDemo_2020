/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.vision.Vision_Tracking;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    // Vision Controller
    private final Vision_Tracking vision = new Vision_Tracking(0);

    // Robot Differential Drive
    private final Drivetrain drivetrain = new Drivetrain(this, vision);

    // Human Controls
    private final Joystick drive_control = new Joystick(0);

    //Ring Light
    private AddressableLED ledCircle;
    private AddressableLEDBuffer ledBuffer;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
    
      //LED Ring Light
      ledCircle = new AddressableLED(0);

      // Default to a length of 16, start empty output
      ledBuffer = new AddressableLEDBuffer(16);
      ledCircle.setLength(ledBuffer.getLength());
  
      // Set the data
      ledCircle.setData(ledBuffer);
      ledCircle.start();

      //Start Vision Thread
      vision.start();
    
    }


    @Override
    public void autonomousInit() {
    }


    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
         // Sets the specified LED to the RGB values for green
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            
            ledBuffer.setRGB(i, 0, 150, 0);
         }
         
         ledCircle.setData(ledBuffer);
    }

    @Override
    public void teleopPeriodic() {
        //Use joysticks to drive, if targeting is disabled
        if(!drivetrain.isTargetingEnabled()){
            drivetrain.set(drive_control.getRawAxis(1), drive_control.getRawAxis(5));
        }
        //Turn on/off camera enabled
        if(drive_control.getRawButton(1)){
            drivetrain.enable_target_tracking(0);
        }else{
            drivetrain.disable_target_tracking();
        }

        //Update driver class
        drivetrain.update();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
