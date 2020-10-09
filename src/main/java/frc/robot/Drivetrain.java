/**
 * Class for controlling the primary functions of a drivetrain
 */

package frc.robot;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.vision.Vision_Tracking;


class Drivetrain{
    
    // Drive Motors
    private final PWMVictorSPX left_drive_1 = new PWMVictorSPX(1);
    private final PWMVictorSPX left_drive_2 = new PWMVictorSPX(2);
    private final PWMVictorSPX right_drive_1 = new PWMVictorSPX(3);
    private final PWMVictorSPX right_drive_2 = new PWMVictorSPX(4);

    // Drive Motor Groups
    private final SpeedControllerGroup left_drive = new SpeedControllerGroup(left_drive_1, left_drive_2);
    private final SpeedControllerGroup right_drive = new SpeedControllerGroup(right_drive_1, right_drive_2);

    RobotBase owner;



    // Tracking Status
    Vision_Tracking vision;
    PIDController pid;
    boolean tracking_enabled = false;
    double target = 0;
    double left_power = 0;
    double right_power = 0;

    private double visionOffset = 0;

    public Drivetrain(RobotBase owner, Vision_Tracking vision) {
        this.owner = owner;
        this.vision = vision;
        
        // Init PID
        pid = new PIDController(.004, .0001, .001);
    }
    
    //Set Drive Motors
    public void set(double left, double right){
        left_drive.set(left);
        right_drive.set(right);
    }

    //Enable Camera Tracking
    public void enable_target_tracking(double offset){
        visionOffset = offset;
        tracking_enabled = true;
    }

    //Disable Camera Tracking
    public void disable_target_tracking(){
        tracking_enabled = false;
    }

    //Get Tracking value
    public boolean isTargetingEnabled(){
        return tracking_enabled;
    }

    //Drive train Update
    public void update(){
        if(owner.isEnabled() && tracking_enabled){
            cameraPIDUpdate();
        }
        SmartDashboard.putBoolean("Track Found", vision.get_foundResults());
        SmartDashboard.putNumber("Center point", vision.get_angleResults());
    
    }

    //Camera Anagle PID Update
    public void cameraPIDUpdate() {
        double output = pid.calculate(vision.get_angleResults(), visionOffset);

        // Set value to rotate the robot
        double left = output / 2;
        double right = output / 2;
        
        // Set linear motion value
        left += left_power;
        right += left_power;

        // Ensure left and right values are between -1 and 1. Rotation prioritized
        if(left > 1){
            right += left - 1;
            left = 1;
        }
        
        if(left < -1){
            right -= left + 1;
            left = 1;
        }

        if(right > 1){
            left += right - 1;
            right = 1;
        }
        
        if(right < -1){
            left -= right + 1;
            right = 1;
        }

        //Set drive motors
        set(left,right);
    }
}