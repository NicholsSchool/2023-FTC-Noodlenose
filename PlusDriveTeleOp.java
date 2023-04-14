package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import java.util.concurrent.TimeUnit;

@TeleOp( name = "PlusDriveTeleOp" )
public class PlusDriveTeleOp extends OpMode
{
    PlusDriveHardwareMap robot = new PlusDriveHardwareMap();
   
    // Constants for automatic arm heights
    private static final double ARM_CONE = 1.919;
    private static final double ARM_HIGH = 1.823;
    private static final double ARM_MID = 1.812;
    
    // Constants for switching speed modes
    private static final double TURN_FAST = 0.75;
    private static final double TURN_SLOW = 0.175;
    private static final double ARM_FAST = 1;
    private static final double ARM_SLOW = 0.5;
    private static final double SPEED_FAST = 1;
    private static final double SPEED_SLOW = 0.35;

    private double TURN_MULTIPLIER;
    private double SPEED_MULTIPLIER;
    
    // Odometry variables
    private double x;
    private double y;
    private double angle;
    
    // Motor speed variables [Front, Back, Left, Right]
    private double fSpeed;
    private double bSpeed;
    private double lSpeed;
    private double rSpeed;

    // Togglable drive modes 
    private boolean isSolo;
    private boolean isArcade;
    private boolean isField;
    
    // Extra - Lights flash red/blue when arm dropping
    private boolean lightFlash;
    
    // Timed claw variables
    private boolean clamped;
    private boolean pressed;
    private long timeCheck;
    
    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void init() 
    {
        // Initialize the hardware variables
        // The init() method of the hardware class does all the work here
        robot.init( hardwareMap );

        // Send telemetry message to signify robot waiting
        telemetry.addData( "Welcome to ", "NOODLENOSE 12737" );
        telemetry.addData( "STATUS: ", "ARMED" );
        
        // Sets variables to defaults
        // Inits into Field Oriented Drive
        isSolo = false;
        isArcade = false;
        isField = true;
        
        lightFlash = true;

        TURN_MULTIPLIER = TURN_FAST;
        SPEED_MULTIPLIER = SPEED_FAST;
        
        clamped = true;
        pressed = true;
                
        robot.RightClaw.setPosition( 1 );
        robot.LeftClaw.setPosition( 0 );
        robot.resetEncoders();
    }

    /* Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY */
    @Override
    public void init_loop() 
    {}

    /* Code to run ONCE when the driver hits PLAY */
    @Override
    public void start() 
    {}

    /* Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP */
    @Override
    public void loop() 
    {
        robot.odometryUpdater();
        switching();
        lighting();
        
        // DO NOT USE RUN_WITH_ENCODERS if using ODOMETRY
        robot.Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Display Driver Information
        telemetry.addData( "Drive Setting: ", getDriveSetting() );
        telemetry.addData( "Coords [X,Y] ", (String)( (int)robot.getX() + ", " + (int)robot.getY() ) );
        telemetry.addData( "Angle: ", robot.angle );
        telemetry.addData( "Arm Height: ", robot.getArmHeight() );
        telemetry.addData( "Four Bar: ", (int)( robot.FourBar.getPosition() * 100 ) / 100.0 ); // Just use .printf() instead
    }
    
    /** 
     * Drives Robot Oriented 
     * - Basic X-Drive/Mecanum trigonometry
     **/
    private void arcadeMode()
    {
        // Ensure motor power is only being set once, if set multiple times motors will be jerky and slow
        fSpeed = SPEED_MULTIPLIER * ( gamepad1.left_stick_x + gamepad1.right_stick_x );
        bSpeed = SPEED_MULTIPLIER * ( gamepad1.left_stick_x + -gamepad1.right_stick_x );
        lSpeed = SPEED_MULTIPLIER * ( -gamepad1.left_stick_y + gamepad1.right_stick_x );
        rSpeed = SPEED_MULTIPLIER * ( -gamepad1.left_stick_y + -gamepad1.right_stick_x );
        
        robot.Front.setPower( fSpeed );
        robot.Back.setPower( bSpeed);
        robot.Left.setPower( lSpeed);
        robot.Right.setPower( rSpeed );
        
        arm();
    }
    
    /** 
     * Drives Field Oriented with Driver and Operator controllers
     * - Joystick forward always drives robot towards direction of initalization
     **/
    private void fieldOriented()
    {
        // Updates trigonometry based on current angle
        double sin = Math.sin( Math.toRadians( robot.getHeading() ) );
        double cos = Math.cos( Math.toRadians( robot.getHeading() ) );
        
        // X-Drive code but relative to sine and cosine values of angle to get Field Oriented
        fSpeed = ( SPEED_MULTIPLIER * ( sin * -gamepad1.left_stick_y + cos * gamepad1.left_stick_x ) + TURN_MULTIPLIER * gamepad1.right_stick_x );
        bSpeed = ( SPEED_MULTIPLIER * ( sin * -gamepad1.left_stick_y + cos * gamepad1.left_stick_x ) + TURN_MULTIPLIER * -gamepad1.right_stick_x );
        rSpeed = ( SPEED_MULTIPLIER * ( cos * -gamepad1.left_stick_y + sin * -gamepad1.left_stick_x ) + TURN_MULTIPLIER * -gamepad1.right_stick_x );
        lSpeed = ( SPEED_MULTIPLIER * ( cos * -gamepad1.left_stick_y + sin * -gamepad1.left_stick_x ) + TURN_MULTIPLIER * gamepad1.right_stick_x );
        
        robot.Front.setPower( fSpeed );
        robot.Back.setPower( bSpeed );
        robot.Left.setPower( lSpeed );
        robot.Right.setPower( rSpeed );
        
        arm();
    }

    /** 
     * Drives Field Oriented but all controls are operated on Driver controller 
     **/
    private void soloMode()
    {
        double sin = Math.sin( Math.toRadians( robot.getHeading() ) );
        double cos = Math.cos( Math.toRadians( robot.getHeading() ) );
        
        fSpeed = ( SPEED_MULTIPLIER * ( sin * -gamepad1.left_stick_y + cos * gamepad1.left_stick_x ) + TURN_MULTIPLIER * gamepad1.right_stick_x );
        bSpeed = ( SPEED_MULTIPLIER * ( sin * -gamepad1.left_stick_y + cos * gamepad1.left_stick_x ) + TURN_MULTIPLIER * -gamepad1.right_stick_x );
        rSpeed = ( SPEED_MULTIPLIER * ( cos * -gamepad1.left_stick_y + sin * -gamepad1.left_stick_x ) + TURN_MULTIPLIER * -gamepad1.right_stick_x );
        lSpeed = ( SPEED_MULTIPLIER * ( cos * -gamepad1.left_stick_y + sin * -gamepad1.left_stick_x ) + TURN_MULTIPLIER * gamepad1.right_stick_x );
        
        robot.Front.setPower( fSpeed );
        robot.Back.setPower( bSpeed );
        robot.Left.setPower( lSpeed );
        robot.Right.setPower( rSpeed );
        
        robot.Arm1.setPower( ( gamepad1.left_trigger - gamepad1.right_trigger ) );
        robot.Arm2.setPower( ( gamepad1.left_trigger - gamepad1.right_trigger ) );
        
        if( gamepad1.x )
            armToPosition( ARM_CONE );
        if( gamepad1.y )
            armToPosition( ARM_HIGH );
        if( gamepad1.b )
            armToPosition( ARM_MID );
            
        fourBar();
        claw();
    }
    
    /**
     * Controls arm
     */
    private void arm()
    {
        // Multiplier to make variable speeds
        robot.Arm1.setPower( ARM_FAST * ( gamepad2.left_trigger - gamepad2.right_trigger ) );
        robot.Arm2.setPower( ARM_FAST * ( gamepad2.left_trigger - gamepad2.right_trigger ) );
        
        if( gamepad2.x )
            armToPosition( ARM_CONE );
        if( gamepad2.y )
            armToPosition( ARM_HIGH );
        if( gamepad2.b )
            armToPosition( ARM_MID );
        
        claw();
        fourBar();
        
        // Auto closes claw
        if( robot.getArmHeight() > 1.83 && robot.getArmHeight() < 1.86 )
        {
            clamped = true;
            claw();
        }
    }
    
    /**
     * When button is held, arm goes to specified position
     * - Set button mapping in Arm method
     * - Uses difference between current and desired heights to create smoothness of PID without extra classes/methods
     * @param voltage height value of arm to go to, Analog Sensors only output a voltage
     */
    private void armToPosition( double voltage )
    {
        robot.Arm1.setPower( ( voltage - robot.getArmHeight() ) * 50 );
        robot.Arm2.setPower( ( voltage - robot.getArmHeight() ) * 50 );
        
        if( lightFlash && voltage == ARM_CONE )
        {
            robot.patternBlinkin( RevBlinkinLedDriver.BlinkinPattern.BLUE );
            lightFlash = false;
        }
        else if( !lightFlash && voltage == ARM_CONE )
        {
            robot.patternBlinkin( RevBlinkinLedDriver.BlinkinPattern.RED );
            lightFlash = true;
        }
        
        if( robot.getArmHeight() > 1.83 && robot.getArmHeight() < 1.86 )
        {
            clamped = true;
            claw();
        }
    }
    
    /**
     * Auto controls Four Bar
     */
    private void fourBar()
    {
        // Auto sets Four Bar position to level based on arm height
        robot.FourBar.setPosition( ( robot.getArmHeight() - 1.790 ) / 0.135 + 0.05 );
    }
    
    /**
     * Button toggle to open/close claw with 0.5 second timer between presses
     */
    private void claw()
    {
        // Makes 0.5 second limit between button presses
        if( gamepad2.a && pressed || gamepad1.a && pressed && isSolo )
        {
            clamped = !clamped;
            pressed = false;
            timeCheck = System.currentTimeMillis();
        }
        
        if( !pressed )
            if( System.currentTimeMillis() - timeCheck > 500 )
                pressed = true;
        
        if( clamped )
        {
            robot.RightClaw.setPosition( 1 );
            robot.LeftClaw.setPosition( 0 );
        }

        // Ground level claw opens wider for ease of cone grabbing
        if( !clamped && robot.getArmHeight() < 1.87 )
        {
            robot.RightClaw.setPosition( .75 );
            robot.LeftClaw.setPosition( .10 );
        }
        else if( !clamped )
        {
            robot.RightClaw.setPosition( .65 );
            robot.LeftClaw.setPosition( .15 );
        }
    }
    
    /**
     * Controls modes based on gamepad buttons pressed
     * - Called by main Loop method -> calls Drive methods
     */
    private void switching()
    {
        // D-Pad controls drive modes
        if( gamepad1.dpad_right )
        {
            isSolo = true;
            isArcade = false;
            isField = false;
        }
        if( gamepad1.dpad_left )
        {
            isSolo = false;
            isArcade = true;
            isField = false;
        }
        if( gamepad1.dpad_up )
        {
            isSolo = false;
            isArcade = false;
            isField = true;
        }
        
        // Bumpers control speed modes
        if( gamepad1.right_bumper )
        {
            TURN_MULTIPLIER = TURN_SLOW;
            SPEED_MULTIPLIER = SPEED_SLOW;
        }
        if( gamepad1.left_bumper )
        {
            TURN_MULTIPLIER = TURN_FAST;
            SPEED_MULTIPLIER = SPEED_FAST;
        }
            
        if( isSolo )
            soloMode();
        else if( isArcade )
            arcadeMode();
        else    
            fieldOriented();
    }
    
    /**
     * Sets color of LED lights
     */
    private void lighting()
    {
        if( gamepad1.x || gamepad2.dpad_left )
            robot.patternBlinkin( RevBlinkinLedDriver.BlinkinPattern.BLUE );
        else if( gamepad1.y || gamepad2.dpad_up )
            robot.patternBlinkin( RevBlinkinLedDriver.BlinkinPattern.YELLOW );
        else if( gamepad1.a || gamepad2.dpad_down )
          robot.patternBlinkin( RevBlinkinLedDriver.BlinkinPattern.GREEN );
        else if( gamepad1.b || gamepad2.dpad_right )
            robot.patternBlinkin( RevBlinkinLedDriver.BlinkinPattern.RED );
    }
    
    /**
     * For displaying info on telemetry
     * @return current Drive mode
     */
    private String getDriveSetting()
    {
        if( isSolo )
            return "Solo - Field Oriented";
        else if( isArcade )
            return "Arcade Drive";
        else
            return "Duo - Field Oriented";
    }
    
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() 
    {}
}
