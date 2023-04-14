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
 * This class intakes and analyzes the camera feed to find if the robot is aligned with the pole
 * 
 * @author Zach Boeck
 * @date April 11th, 2023
 */
public class PolePipeline extends OpenCvPipeline
{
    // HSV Color Value ( Hue [0,180], Saturation [0,255], Value [0,255] )
    Scalar YELLOW_MAX = new Scalar( 20, 255, 255 );
    Scalar YELLOW_MIN = new Scalar( 10, 125, 125 );
    
    // Camera resolution is 800 x 448
    Rect leftRect = new Rect( new Point(230,0), new Point(430,224) );
    Rect rightRect = new Rect( new Point(430,0), new Point(630,224) );
    
    Mat yellow;
    
    private double leftAMT = 0;
    private double rightAMT = 0;
    
    // Minimum value for lighting/interference differences
    private final double COLOR_AMT = 1000;
    
    /**
     * This converts the image and extracts the amount of yellow color on the left and right side of the camera
     * @param input Input frame from camera
     * @return Image of only the yellow objects of the image
     */
    @Override
    public Mat processFrame( Mat input )
    {
        yellow = input;
        
        // Converts frame from RGBA to HSV
        Imgproc.cvtColor( yellow, yellow, Imgproc.COLOR_RGBA2RGB );
        Imgproc.cvtColor( yellow, yellow, Imgproc.COLOR_RGB2HSV );
        
        // Removes all color values except yellow
        Core.inRange( yellow, YELLOW_MIN, YELLOW_MAX, yellow );
        
        Mat left = yellow;
        Mat right = yellow;
        
        // Crops into small target areas on left and right side of camera
        left = yellow.submat( leftRect );
        right = yellow.submat( rightRect );
        
        // Extracts yellow values on both sides
        leftAMT = Core.sumElems( left ).val[0];
        rightAMT = Core.sumElems( right ).val[0];
        
        // Adds outline to image view on Driver Station
        Imgproc.rectangle( yellow, leftRect, new Scalar(255,50,50), 5 );
        Imgproc.rectangle( yellow, rightRect, new Scalar(255,50,50), 5 );
        
        return yellow;
    }
    
    /**
     * Decides which direction robot is misaligned with pole in
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
    
}
