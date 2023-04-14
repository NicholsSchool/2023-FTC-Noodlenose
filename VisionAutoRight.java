package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

// Vision Imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous( name = "VisionAutoRight" )
public class VisionAutoRight extends LinearOpMode
{
    private DcMotor Front, Right, Back, Left;
    private DcMotorSimple Arm1, Arm2;
    private Servo FourBar, RightClaw, LeftClaw;
    private AnalogInput ArmSensor;

    private RevBlinkinLedDriver blinkin;
    
    private BNO055IMU imu = null;
    private OpenCvWebcam camera = null;
    private OpenCvWebcam cameraArm = null;
    
    double fSpeed, bSpeed, lSpeed, rSpeed;

    private Odometry odometry;
    double xCoord, yCoord, angle;

    private PolePipeline poleFinder;
    private ConePipeline coneFinder;

    final double DRIVE_SPEED = 0.5;
    double TURN_SPEED = 0.2;
    final double TURN_FAST = 0.5;
    final double TURN_SLOW = 0.2;
    final double ARM_SPEED = 1; // 0.6;
    double END = 1;
   
    // Arm height Constants
    final double HIGH_DROP = 1.818;
    final double HIGH_POLE = 1.828;
    final double VERTICAL = 1.875;
    final double STACK_FLOAT = 1.890;

    final double CONE_1 = 1.906;
    final double CONE_2 = 1.908;
    final double CONE_3 = 1.908;
    final double CONE_4 = 1.911;
    final double CONE_5 = 1.913;
    
    /* April Tag Objects  \/ */

    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    int park;
    boolean found;
    
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.12;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    
    /* April Tag Objects /\ */
    
   
    /* COMMANDS \/ */
    
    /**
     * Uses difference of current and desired arm heights to create smoothness of PID without extra classes/methods
     * @param voltage Arm height constant to go to
     */
    private void arm( double voltage )
    {
        if( ArmSensor.getVoltage() < voltage )
            while( Math.abs( ArmSensor.getVoltage() - voltage ) > 0.002 && opModeIsActive() )
            {
                Arm1.setPower( -( voltage - ArmSensor.getVoltage() ) * 350 );
                Arm2.setPower( -( voltage - ArmSensor.getVoltage() ) * 350 );
                fourBar();
            }
        else if( ArmSensor.getVoltage() > voltage )
            while( Math.abs( ArmSensor.getVoltage() - voltage ) > 0.002 && opModeIsActive() )
            {
                Arm1.setPower( -( voltage - ArmSensor.getVoltage() ) * 350 );
                Arm2.setPower( -( voltage - ArmSensor.getVoltage() ) * 350 );
                fourBar();
            }
        
        Arm1.setPower(0);
        Arm2.setPower(0);
    }
    
    /**
     * Auto sets the Four Bar position to keep it level
     */
    private void fourBar()
    {
        // ( Current - Minimum ) / Distance travelled over 180 degrees
        FourBar.setPosition( ( ArmSensor.getVoltage() - 1.790 ) / 0.135 );
    }
    
    /**
     * Opens/closes the claw
     * @param clamped true if claw is to be closed
     */
    private void claw( boolean clamped )
    {
        if( clamped )
        {
            RightClaw.setPosition( 1 );
            LeftClaw.setPosition( 0 );
        }
        if( !clamped )
        {
            RightClaw.setPosition( .65 );
            LeftClaw.setPosition( .15 );
        }
    }
    
    /**
     * Turns the robot at constant TURN_SPEED
     * @param desiredAngle Angle to rotate to [-180 -> 180]
     */
    private void turn( int desiredAngle )
    {
        while( desiredAngle - angle > 0 && Math.abs( angle - desiredAngle ) > 5 && opModeIsActive() )
        {
            Front.setPower( -TURN_SPEED );
            Right.setPower( TURN_SPEED );
            Back.setPower( TURN_SPEED );
            Left.setPower( -TURN_SPEED );
            
            odometryUpdater();
        }
        stopMotors();
        while( desiredAngle - angle < 0 && Math.abs( angle - desiredAngle ) > 5 && opModeIsActive() )
        {
            Front.setPower( TURN_SPEED );
            Right.setPower( -TURN_SPEED );
            Back.setPower( -TURN_SPEED );
            Left.setPower( TURN_SPEED );
            
            odometryUpdater();
        }
        stopMotors();
    }
    
    /**
     * Drives the robot to X Y coordinate of field [+-1 inch accuracy]
     * @param xLocation Forward position across field [away from driver]
     * @param yLocation Sideways position across field [to the left of driver]
     */
    private void move( double xLocation, double yLocation )
    {
        while( Math.abs( xLocation - xCoord ) > 0.5 || Math.abs( yLocation - yCoord ) > 0.5 && opModeIsActive() )
        {
            double xValue = ( xLocation - xCoord ) / 4;
            double yValue = ( yLocation - yCoord ) / 4;
            drive( Math.min( xValue, 0.5 ), Math.min( yValue, 0.5 ) );
        }
        
        stopMotors();
    }
    
    /**
     * Drives the robot in Field Oriented style
     * - Drives in straight line regardless of interference/rotation
     * - [1,0] Forward [0,1] Left [1,1] Diagonal Forward and Left 
     * @param xPower Power to move in X direction
     * @param yPower Power to move in Y direction
     */
    private void drive( double xPower, double yPower )
    {
        odometryUpdater();

        double sin = Math.sin( Math.toRadians( angle ) );
        double cos = Math.cos( Math.toRadians( angle ) );
        
        fSpeed =  ( sin * xPower + cos * -yPower );
        bSpeed =  ( sin * xPower + cos * -yPower );
        rSpeed =  ( cos * xPower + sin * yPower );
        lSpeed =  ( cos * xPower + sin * yPower );
        
        Front.setPower( fSpeed );
        Back.setPower( bSpeed );
        Left.setPower( lSpeed );
        Right.setPower( rSpeed );
    }
    
    /**
     * Timed Arcade Drive
     * @param sec Duration of drive [seconds]
     * @param power Power to drive at
     */
    private void timeDrive( double sec, double power )
    {
        double start = System.currentTimeMillis();
        
        while( System.currentTimeMillis() - start < sec * 1000 && opModeIsActive() )
        {
            Left.setPower( power );
            Right.setPower( power );
            odometryUpdater();
        }
        
        stopMotors();
    }
    
    /**
     * Drives, turns, and moves arm to specified positions at the same time [+-1 inch][+-3 Degrees]
     * @param xLocation Forward position across field [away from driver]
     * @param yLocation Sideways position across field [to the left of driver]
     * @param desiredAngle Angle to rotate to [-180 -> 180]
     * @param voltage Arm height constant to go to
     */
    private void combo( int xLocation, int yLocation, int desiredAngle, double voltage )
    {
        double fPow;
        double rPow;
        double bPow;
        double lPow;
        
        while( Math.abs( xLocation - xCoord ) > 1 || Math.abs( yLocation - yCoord ) > 1 || 
               Math.abs( angle - desiredAngle ) > 5 || Math.abs( ArmSensor.getVoltage() - voltage ) > 0.003 )
        {
            odometryUpdater();
            
            // Adds power to variable and sets motor power once at end
            fPow = 0;
            rPow = 0;
            bPow = 0;
            lPow = 0;
            
            // Move + Drive
            if( Math.abs( xLocation - xCoord ) > 0.5 || Math.abs( yLocation - yCoord ) > 0.5 )
            {
                double xPower = Math.min( ( xLocation - xCoord ) / 4, 0.4 );
                double yPower = Math.min( ( yLocation - yCoord ) / 4, 0.4 );
                
                double sin = Math.sin( Math.toRadians( angle ) );
                double cos = Math.cos( Math.toRadians( angle ) );
        
                fPow += END * ( sin * xPower + cos * -yPower );
                bPow += END * ( sin * xPower + cos * -yPower );
                rPow += END * ( cos * xPower + sin * yPower );
                lPow += END * ( cos * xPower + sin * yPower );
            }
            
            // Turn
            if( Math.abs( angle - desiredAngle ) > 5 )
            {
                int clockwise;
                if( desiredAngle - angle > 0 )
                    clockwise = -1;
                else if( desiredAngle - angle < 0 )
                    clockwise = 1;
                else
                    clockwise = 0;
                    
                fPow += clockwise * TURN_SPEED / 2;
                rPow += -clockwise * TURN_SPEED / 2;
                bPow += -clockwise * TURN_SPEED / 2;
                lPow += clockwise * TURN_SPEED / 2;
            }
            
            // Arm
            if( Math.abs( ArmSensor.getVoltage() - voltage ) > 0.002 )
            {
                Arm1.setPower( -( voltage - ArmSensor.getVoltage() ) * 100 );
                Arm2.setPower( -( voltage - ArmSensor.getVoltage() ) * 100 );
                fourBar();
            } 
            else
            {
                Arm1.setPower( 0 );
                Arm2.setPower( 0 );
            }
            
            Front.setPower( fPow );
            Right.setPower( rPow );
            Back.setPower( bPow );
            Left.setPower( lPow );
        }
            
        stopMotors();
    }
    
    /**
     * Stops motors
     */
    private void stopMotors()
    {
        Front.setPower(0);
        Right.setPower(0);
        Back.setPower(0);
        Left.setPower(0);
        Arm1.setPower(0);
        Arm2.setPower(0);
    }
    
    /**
     * Vision method to align robot with yellow pole
     */
    private void poleAlign()
    {
        cameraArm.setPipeline( poleFinder );
        
        // 3 second time limit to prevent stuck in loop
        double time = System.currentTimeMillis() + 3000;

        while( System.currentTimeMillis() < time && opModeIsActive() )
        {
            telemetry.addData( "ALIGN: ", poleFinder.direction() );
            telemetry.update();
            
            // Uses -1, 0, 1 alignment data
            if( poleFinder.direction() == -1 )
            {
                Front.setPower( -0.1 );
                Right.setPower( 0.1 );
                Back.setPower( 0.1 );
                Left.setPower( -0.1 );
            
                odometryUpdater();
            }
            stopMotors();
            if( poleFinder.direction() == 1 )
            {
                Front.setPower( 0.1 );
                Right.setPower( -0.1 );
                Back.setPower( -0.1 );
                Left.setPower( 0.1 );
            
                odometryUpdater();
            }
            stopMotors();
            if( poleFinder.direction() == 0 )
            {
                stopMotors();
                time = System.currentTimeMillis();
            }
        }
        stopMotors();

        // Cone dropping methods
        arm( HIGH_DROP );
        sleep(100);
        claw( false );
        sleep(100);
    }
    
    /**
     * Vision method to align with blue and red cones
     */
    private void coneAlign()
    {
        cameraArm.setPipeline( coneFinder );
        
        telemetry.addData( "ALIGN: ", coneFinder.direction() );
        telemetry.addData( "ALLIANCE: ", coneFinder.isBlue() );
        telemetry.update();
        
        claw( false );
            
        double time = System.currentTimeMillis() + 2000;

        while( System.currentTimeMillis() < time && opModeIsActive() )
        {
            telemetry.addData( "ALIGN: ", coneFinder.direction() );
            telemetry.addData( "ALLIANCE: ", coneFinder.isBlue() );
            telemetry.update();
            
            if( coneFinder.direction() == 1 )
            {
                Front.setPower( -0.1 );
                Right.setPower( 0.1 );
                Back.setPower( 0.1 );
                Left.setPower( -0.1 );
            
                odometryUpdater();
            }
            stopMotors();
            if( coneFinder.direction() == -1 )
            {
                Front.setPower( 0.1 );
                Right.setPower( -0.1 );
                Back.setPower( -0.1 );
                Left.setPower( 0.1 );
            
                odometryUpdater();
            }
            stopMotors();
            if( coneFinder.direction() == 0 )
            {
                stopMotors();
                time = System.currentTimeMillis();
            }
        }
        
        stopMotors();
        
        // Sets LEDs to alliance color
        if( coneFinder.isBlue() == 1 )
            blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.BLUE );
        else if( coneFinder.isBlue() == -1 )
            blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.RED );
    }
    
    
    /* Initialization and Command Calls \/ */
    
    @Override
    public void runOpMode() throws InterruptedException
    {
        // initialize stuff
        
        Front = hardwareMap.get(DcMotor.class, "Front");
        Right = hardwareMap.get(DcMotor.class, "Right");
        Back = hardwareMap.get(DcMotor.class, "Back");
        Left = hardwareMap.get(DcMotor.class, "Left");
        
        Arm1 = hardwareMap.get(DcMotorSimple.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotorSimple.class, "Arm2");
        ArmSensor = hardwareMap.get(AnalogInput.class, "ArmSensor");
        
        FourBar = hardwareMap.get(Servo.class, "FourBar");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        
        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        
        Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS); 
        Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS); 
        Back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS); 
        Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS); 
        
        Front.setDirection(DcMotor.Direction.REVERSE); 
        Back.setDirection(DcMotor.Direction.FORWARD);
        Left.setDirection(DcMotor.Direction.REVERSE); 
        Right.setDirection(DcMotor.Direction.FORWARD); 
        
        Arm1.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        
        Front.setPower(0);
        Right.setPower(0);
        Back.setPower(0);
        Left.setPower(0);
        
        // Initializes Odometry with starting offsets
        // 8 inches forward, 36 inches left, 90 degrees counter clockwise
        // 4324 = 8 inches / 0.00185 TICK_CONVERT
        odometry = new Odometry( 4324, 19459, 90 ); 
        xCoord = 0;
        yCoord = 0;
        angle = 0;

        park = 1;
        END = 1;
        TURN_SPEED = TURN_SLOW;
        
        poleFinder = new PolePipeline();
        coneFinder = new ConePipeline();
        
        RightClaw.setPosition( 1 );
        LeftClaw.setPosition( 0 );
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        
        blinkin = hardwareMap.get( RevBlinkinLedDriver.class, "Blinkin" );
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        // Reads April Tags during initialization and before game begins
        aprilTagReader();

        visionInit();
        
        waitForStart();
        
        if( opModeIsActive() )
        {
            /* COMMAND CALLS \/ */
            
            // Starting Cone
            claw( true );
            combo( 60, 36, -135, HIGH_POLE );
            poleAlign();
            sleep(100);
            claw( true );
            
            double[] stack = new double[]{ CONE_1, CONE_2, CONE_3, CONE_4, CONE_5 };
            
            // Repeats same movements for cone stack scoring
            // i < # Cones to score
            for( int i = 0; i < 2; i++ )
            {
                // Cone stack direction
                combo( 60, 36, -85, STACK_FLOAT );
                claw( false );
                arm( stack[i] );

                // Grabs cone
                coneAlign();
                sleep(100);
                timeDrive( 0.5, 0.5 );
                claw( true );
                sleep(200);

                // Scores cone
                combo( 60, 36, -135, HIGH_POLE );
                poleAlign();
                sleep(250);
                claw( true );
            }

            // Limits speed, drive speed at end was wonky/inconsistent
            END = 0.55;
            
            // Parks the robot in X Y position based on April Tags
            // Parks facing away from driver so TeleOp Field Oriented drive initializes correctly
            if( park == 3 )
            { 
                combo( 36, 36, 0, VERTICAL );
                combo( 38, 14, 0, VERTICAL );
            }
            if( park == 2 )
            {
                combo( 36, 36, 0, VERTICAL );
                combo( 38, 36, 0, VERTICAL );
            }
            if( park == 1 )
            {
                combo( 36, 36, 0, VERTICAL );
                combo( 38, 60, 0, VERTICAL );
            }
        }
    }
    
    /**
     * Updates Odometry Coordinates [Call inside running loop to keep updated]
     */
    private void odometryUpdater()
    {
        odometry.update( Front.getCurrentPosition(),
                         Right.getCurrentPosition(),
                         Left.getCurrentPosition(),
                         Back.getCurrentPosition(), 
                         imu.getAngularOrientation().firstAngle );
                         
        // Local variables for efficiency
        xCoord = odometry.getX();
        yCoord = odometry.getY();
        angle = odometry.getAngle();
        
        telemetry.addData( "X: ", xCoord );
        telemetry.addData( "Y: ", yCoord );
        telemetry.addData( "Angle: ", angle );
        telemetry.addData( "Park: ", park );
        telemetry.update();
    }
    
    /**
     * Initializes the Arm Camera for pole and cone vision alignment
     */
    private void visionInit()
    {
        cameraArm = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        poleFinder = new PolePipeline();

        cameraArm.setPipeline( poleFinder );
        cameraArm.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cameraArm.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {}
        });
    }
    
    /**
     * Initializes the Side Camera and April Tag Pipeline
     */
    private void aprilTagReader()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {}
        });
        
        // Calls method to read April Tags
        while( !opModeIsActive() )
        {
            vision();
            sleep(20);
            telemetry.addData( "Park", park );
            telemetry.update();
        }
        
        camera.stopStreaming();
    }
    
    /**
     * Reads the April Tags
     */
    private void vision()
    {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

        if(detections != null)
        {
            if(detections.size() == 0)
            {
                numFramesWithoutDetection++;

                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
            }
            else
            {
                numFramesWithoutDetection = 0;

                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }

                for(AprilTagDetection detection : detections)
                {
                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    // park = detection.id;
                    if( detection.id == 1 || detection.id == 2 || detection.id == 3 )
                    {
                        park = detection.id;
                        found = true;
                    }
                }
            }

            telemetry.update();
        }
    }
}

