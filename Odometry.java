package org.firstinspires.ftc.teamcode;

/**
 * This class controls the trigonometry and data of the odometry system
 * - This keeps track of position by subtracting the previous encoder values from the current encoder values
 * - This finds the change in values without requiring the STOP_AND_RESET_ENCODERS method call
 * - That call causes motors to constantly stop, creating slow, noisy, and shaky driving
 * 
 * @author Zach Boeck
 * @date April 11th, 2023
 */
public class Odometry 
{
    // Inches / Ticks    ( Circumference of wheel / Ticks per revolution ( 2048 ) )
    private static final double TICK_CONVERT = 0.00185;
    
    private double angleOffset;
    private double trueAngle;
    
    // X and Y position stored in Ticks
    private double x; 
    private double y;

    // Previous values of encoders
    private double lastFR;
    private double lastBR;
    private double lastFL;
    private double lastBL;

    // Change in value of encoders
    private double FR; // Front Right
    private double BR; // Back Right
    private double FL; // Front Left
    private double BL; // Back Left
    
    /**
     * Initializes the odometry data and positioning
     * 
     * @param startX X location on field robot initializes at
     * @param startY Y location on field robot intializes at
     * @param startAngleOffset Angle relative to driver robot initializes at
     */
    public Odometry( int startX, int startY, int startAngleOffset )
    {
        x = startX;
        y = startY;
        angleOffset = startAngleOffset;
    }
    
    /**
     * Updates the odometry data and positioning using current and previous values of the encoders
     * 
     * @param newFR Current value of the Front Right encoder
     * @param newBR Current value of the Back Right encoder
     * @param newFL Current value of the Front Left encoder
     * @param newBL Current value of the Back Left encoder
     * @param angle Current value of the IMU angle
     */
    public void update( double newFR, double newBR, double newFL, double newBL, double angle )
    {
        newFR = -newFR;
        newBL = -newBL;
        
        // Finds change in value
        FR = newFR - lastFR;
        BR = newBR - lastBR;
        FL = newFL - lastFL;
        BL = newBL - lastBL;
        
        // Averages parallel encoders
        double leftDiag = ( FR + BL ) / 2;
        double rightDiag = ( BR + FL ) / 2;
        angle += angleOffset;
        
        // Angle offset combined with -180 -> 180 range creates blindspots
        // This checks for and removes the blindspots
        if( angle < 181 && angle > -181 )
            trueAngle = angle;
        if( angle > 180 )
            trueAngle = -180 + angle % 180;
        else if( angle < -180 )
            trueAngle = 180 + angle % 180;
        
        // Adds the sine and cosine values to the current position
        x += ( Math.cos( Math.toRadians( angle + 45 ) ) * leftDiag + 
               Math.cos( Math.toRadians( angle - 45 ) ) * rightDiag ) / 2;
        y += ( Math.sin( Math.toRadians( angle + 45 ) ) * leftDiag + 
               Math.sin( Math.toRadians( angle - 45 ) ) * rightDiag ) / 2;
        
        lastFR = newFR;
        lastBR = newBR;
        lastFL = newFL;
        lastBL = newBL;
    }
    
    /**
     * Returns the X location of the robot in Inches
     * @return X Coord of robot in Inches
     */
    public double getX()
    {
        return x * TICK_CONVERT;
    }
    
    /**
     * Returns the Y location of the robot in Inches
     * @return Y Coord of robot in Inches
     */
    public double getY()
    {
        return y * TICK_CONVERT;
    }
    
    /**
     * Returns the oriented angle of the robot ( with starting offset included )
     * @return Field oriented angle of the robot
     */
    public double getAngle()
    {
        return trueAngle;
    }
}