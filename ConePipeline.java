package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

/**
 * This class intakes and analyzes the camera feed to find if the robot is aligned with a stack of cones
 * 
 * @author Zach Boeck
 * @date April 11th, 2023
 */
public class ConePipeline extends OpenCvPipeline
{
    // HSV Color Value ( Hue [0,180], Saturation [0,255], Value [0,255] )
    Scalar RED_MAX = new Scalar( 5, 255, 255 );
    Scalar RED_MIN = new Scalar( 0, 125, 125 );
    
    Scalar BLUE_MAX = new Scalar( 110, 255, 255 );
    Scalar BLUE_MIN = new Scalar( 95, 90, 90 );
    
    // Camera resolution is 800 x 448
    // 30 pixel offset to the right to even the camera
    Rect leftRect = new Rect( new Point(230,112), new Point(430,448) );
    Rect rightRect = new Rect( new Point(430,112), new Point(630,448) );
    
    Mat red;
    Mat blue;
    
    private int isBlue = 0;
    
    private double leftAMT = 0;
    private double rightAMT = 0;
    
    public double redAMT = 0;
    public double blueAMT = 0;
    
    private double leftRED;
    private double rightRED;
    private double leftBLUE;
    private double rightBLUE;
    
    // Minimum value for lighting/interference differences
    private final double COLOR_AMT = 1000;
    
    private boolean turn = true;
    private int count = 0;
    
    /**
     * This class finds whether the robot is on Red or Blue alliance and calls the proccessing of the images
     * @param input Input frame from camera
     * @return Unedited input frame from camera
     */
    @Override
    public Mat processFrame( Mat input )
    {
        // First 4 frames are used for alliance selection
        if( count < 4 )
        {
            count++;
            if( turn )
            {
                proccessBlue( input );
                turn = false;
            }
            else
            {
                proccessRed( input );
                turn = true;
            }
        }
        
        // Sets alliance
        if( count >= 4 && count < 100 )
        {
            if( blueAMT > redAMT )
                isBlue = 1;
            else if( blueAMT < redAMT )
                isBlue = -1;
            else
                isBlue = 0;
            count = 200;
        }
        
        // Calls frames to be proccessed based on alliance color
        if( isBlue == 1 )
            return proccessBlue( input );
        else if( isBlue == -1 )
            return proccessRed( input );
        
        return input;
    }
    
    /**
     * This converts the image and extracts the amount of red color on the left and right side of the camera
     * @param input Input frame from camera
     * @return Image of only the red objects of the image
     */
    public Mat proccessRed( Mat input )
    {
        red = input;
        
        // Converts frame from RGBA to HSV
        Imgproc.cvtColor( red, red, Imgproc.COLOR_RGBA2RGB );
        Imgproc.cvtColor( red, red, Imgproc.COLOR_RGB2HSV );
        
        // Removes all color values except red
        Core.inRange( red, RED_MIN, RED_MAX, red );
        
        Mat left = red;
        Mat right = red;
        
        // Crops into small target areas on left and right side of camera
        left = red.submat( leftRect );
        right = red.submat( rightRect );
        
        // Extracts red values on both sides
        leftAMT = Core.sumElems( left ).val[0];
        rightAMT = Core.sumElems( right ).val[0];
        
        redAMT = leftAMT + rightAMT;
        
        // Adds outline to image view on Driver Station
        Imgproc.rectangle( red, leftRect, new Scalar(255,50,50), 5 );
        Imgproc.rectangle( red, rightRect, new Scalar(255,50,50), 5 );
        
        return red;
    }
    
    /**
     * This converts the image and extracts the amount of blue color on the left and right side of the camera
     * @param input Input frame from camera
     * @return Image of only the red objects of the image
     */
    public Mat proccessBlue( Mat input )
    {
        blue = input;
        
        // Converts frame from RGBA to HSV
        Imgproc.cvtColor( blue, blue, Imgproc.COLOR_RGBA2RGB );
        Imgproc.cvtColor( blue, blue, Imgproc.COLOR_RGB2HSV );
        
        // Removes all color values except red
        Core.inRange( blue, BLUE_MIN, BLUE_MAX, blue );
        
        Mat left = blue;
        Mat right = blue;
        
        // Crops into small target areas on left and right side of camera
        left = blue.submat( leftRect );
        right = blue.submat( rightRect );
        
        // Extracts blue values on both sides
        leftAMT = Core.sumElems( left ).val[0];
        rightAMT = Core.sumElems( right ).val[0];
        
        blueAMT = leftAMT + rightAMT;
        
        // Adds outline to image view on Driver Station
        Imgproc.rectangle( blue, leftRect, new Scalar(255,50,50), 5 );
        Imgproc.rectangle( blue, rightRect, new Scalar(255,50,50), 5 );
        
        return blue;
    }
    
    /**
     * Decides which direction robot is misaligned with cone stack in
     * @return 0 if aligned, 1 if too far left, -1 if too far right, 2 if error
     */
    public int direction()
    {
        if( leftAMT + rightAMT > COLOR_AMT )
        {
            if( Math.max( leftAMT, rightAMT ) / Math.min( leftAMT, rightAMT ) < 1.5 )
                return 0;
            else if( leftAMT - rightAMT > 0 )
                return 1;
            else if( leftAMT - rightAMT < 0 )
                return -1;
        }
        return 2;
    }
    
    /**
     * Returns the alliance of the robot
     * @return true if on blue alliance, false if on red alliance
     */
    public int isBlue()
    {
        return isBlue;
    }
}
