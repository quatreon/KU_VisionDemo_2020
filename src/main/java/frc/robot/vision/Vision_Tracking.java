/**
 * Contains all the processing done outside of the GRIP pileline
 */

package frc.robot.vision;

import java.util.ArrayList;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import frc.robot.vision.GripPipelineTwo;

/**
 * Manages all the vision tracking operations in a seperate thread
 */
public class Vision_Tracking implements Runnable{
    public enum Tracking_Param {
        HORIZONTAL,
        VERTICAL
    }

    // Calculation Paramters
    private static final int IMG_WIDTH = 1080;          // Width of the image
    private static final int IMG_HEIGHT = 1920;          // Height of the image
    private final double ang_per_px = 60.0/(double)IMG_WIDTH;     // Angle represented by each pixel
    
    // Vision Object
    private int camera_index;                           // Index of the camera used for the vision tracking
    private UsbCamera camera;                           // Camera object used for vision tracking
    private CvSink camera_sink;                         // Camera Sink object used for vision tracking
    private Mat camera_image;               // Image buffer for image captured for vision tracking
    private GripPipelineTwo grip_pipeline;                 // Grip Pipeline
    private Thread vision_thread;                       // Thread used for vision tracking
    
    // Results
    private ArrayList<Rect> target_pairs = new ArrayList<Rect>();
    private final Object result_lock = new Object();    // Thread Lock object
    
    // Extra Video Feeds for testing
    private CvSource hsv_threashold_source;
    private CvSource erode_source;


    /**
     * Vision Result container class
     */
    class Vision_Result{
        public boolean found = false;
        public double centerLeft = 0;
        public double centerRight = 0;
    }

    /**
     * Constructor
     * @param camera_index  Index of the camera to use for the vision tracking
     * @param ang_per_px    Angle represented by each pixel of the camera
     */
    public Vision_Tracking(int camera_index) {
        // Init Camera
        this.camera_index = camera_index;
        camera = CameraServer.getInstance().startAutomaticCapture(camera_index);
        camera_sink = CameraServer.getInstance().getVideo();
        camera_image = new Mat();

        //Set camera resolution, only if camera was succesfully initalized
        if(camera != null){
            camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        }

         //Int Extra Video Feeds for testing
        hsv_threashold_source = CameraServer.getInstance().putVideo("HSV Threshold", 480, 640);
		erode_source = CameraServer.getInstance().putVideo("Erode", 480, 640);
        
        // Init Vision Pipeline
        grip_pipeline = new GripPipelineTwo();

        // Init Vision Thread
        vision_thread = new Thread(this);
    }

    /**
     * Starts the Vision Processing thread
     */
    public void start(){
        vision_thread.start();
    }

    /**
     * Stops the Vision Processing thread
     * 
     * @param timeout timeout for waiting for the thread in milliseconds. Set to less than 0, timeout will be infinite.
     * @throws InterruptedException
     */
    public void stop(long timeout) throws InterruptedException {
        vision_thread.interrupt();

        if(timeout >= 0){
            vision_thread.join(timeout);
        }else{
            vision_thread.join();
        }
    }

    /**
     * Sorts the found vision target objects and matches target pairs
     */
    private void sort_targets(){
        ArrayList<Rect> left_rectangles = new ArrayList<Rect>();
        ArrayList<Rect> right_rectangles = new ArrayList<Rect>();
        ArrayList<MatOfPoint> contours = grip_pipeline.filterContoursOutput();

        // Check if no targets are found
        if(!contours.isEmpty()){
            int numBoxes = contours.size();
                
            //Find target angle and center point for each contour
            for(int count = 0; count < numBoxes; count++) {
                
                MatOfPoint2f tempMat = new MatOfPoint2f(contours.get(count).toArray());
                RotatedRect tempAngle = Imgproc.minAreaRect(tempMat);
                Rect tempRec = Imgproc.boundingRect(contours.get(count));
                //System.out.println("Target: " + count + " x: " + tempRec.x + " Angle: " + tempAngle.angle + "Area: " + tempRec.area() + " Height:" + tempRec.height);
                
                //Is this a right target? Angle is about -14.
                //Or is this a left target? Angle is about -75.
                if(tempAngle.angle < 0 && tempAngle.angle > -45) {
                    right_rectangles.add(tempRec);
                }else if(tempAngle.angle < -45 && tempAngle.angle > -100) {
                    left_rectangles.add(tempRec);
    
                }
            }
         }

            //Get biggest right\left target, make them avialable for main program to use
            //Note: this does not do left and right target matching
            synchronized (result_lock) {
                target_pairs.clear();
                if(!left_rectangles.isEmpty()){
                    target_pairs.add(left_rectangles.get(0));  
                }
                if(!right_rectangles.isEmpty()){
                    target_pairs.add(right_rectangles.get(0)); 
                }
            }
       
    }

    /**
     * Thread processing method
     */
    public void run(){
        while(!Thread.interrupted()){
            // Retrieve Camera 
            long result = camera_sink.grabFrameNoTimeout(camera_image);
                    
            if(result != 0){
                // Run GRIP Pipeline
                grip_pipeline.process(camera_image);

                // Sort targets
                sort_targets();
            } else{
                synchronized(result_lock){
                    target_pairs.clear();
                }
            }

            //Output processed video to smartdash board: Not requried, but useful for debugging
			 hsv_threashold_source.putFrame(grip_pipeline.hsvThresholdOutput());
             erode_source.putFrame(grip_pipeline.cvErodeOutput());
           
        }
    }

    //Get final results
    public Vision_Result get_result(){
        Vision_Result result = new Vision_Result();

        synchronized (result_lock) {
            result.found = !target_pairs.isEmpty();

            if(target_pairs.size()>=2){
                double centerLeft = target_pairs.get(0).x;
                double centerRight = target_pairs.get(1).x;
                result.centerLeft = centerLeft;
                result.centerRight = centerRight;
            }
        }
        return result;
    }

    //Get calcualted angle from final results
    public double get_angleResults() {
        double result = 0;
        Vision_Result vision_result = get_result();
        double centerX = (vision_result.centerRight + vision_result.centerLeft)/2.0;
        //How far are the results from the center of the image? Time that by pixels per degree
        result = ang_per_px * (centerX - IMG_WIDTH / 2);

        return result;
    }

    //Get "is target found" information from final results
     public boolean get_foundResults() {
        Vision_Result vision_result = get_result();
        return vision_result.found;
    }


}