package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.concurrent.TimeUnit;

public class PlusDriveHardwareMap
{
    // Motors and Sensors
    public DcMotor  Front, Back, Left, Right;
    public DcMotorSimple Arm1, Arm2;
    public Servo FourBar, RightClaw, LeftClaw;
    public AnalogInput ArmSensor;

    public BNO055IMU imu = null;
    public RevBlinkinLedDriver Blinkin = null;
    
    public Odometry odometry;
    public double armHeight;
    
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public PlusDriveHardwareMap()
    {}

    /* Initialize standard Hardware interfaces */
    public void init( HardwareMap ahwMap ) 
    {
        // Save reference to Hardware map
        HardwareMap hwMap = ahwMap;

        // Define and Initialize Motors
        Front  = hwMap.get(DcMotor.class, "Front");
        Back = hwMap.get(DcMotor.class, "Back");
        Left    = hwMap.get(DcMotor.class, "Left");
        Right    = hwMap.get(DcMotor.class, "Right");
        
        Arm1 = hwMap.get(DcMotorSimple.class, "Arm1");
        Arm2 = hwMap.get(DcMotorSimple.class, "Arm2");
        ArmSensor = hwMap.get(AnalogInput.class, "ArmSensor");

        RightClaw = hwMap.get(Servo.class, "RightClaw");
        LeftClaw = hwMap.get(Servo.class, "LeftClaw");
        FourBar = hwMap.get(Servo.class, "FourBar");
        
        Blinkin = hwMap.get( RevBlinkinLedDriver.class, "Blinkin" );
        
        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        Front.setDirection(DcMotor.Direction.REVERSE);
        Back.setDirection(DcMotor.Direction.FORWARD);
        Left.setDirection(DcMotor.Direction.REVERSE);
        Right.setDirection(DcMotor.Direction.FORWARD);
        
        Arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm2.setDirection(DcMotorSimple.Direction.FORWARD);

        Front.setPower(0);
        Back.setPower(0);
        Left.setPower(0);
        Right.setPower(0);
        
        Arm1.setPower(0);
        Arm2.setPower(0);
        
        odometry = new Odometry( 0, 0, 0 );
        armHeight = 0;
    }
    
    /**
     * Sets the color of the LEDs
     * @param pattern Color to set LEDs to
     */
    public void patternBlinkin( RevBlinkinLedDriver.BlinkinPattern pattern )
    {
        Blinkin.setPattern( pattern );
    }
    
    /**
     * Gets the angle from the IMU
     * @return Angle the robot is at [-180 -> 180]
     */
    public float getHeading()
    {
       return imu.getAngularOrientation().firstAngle; 
    }
    
    /**
     * Gets the height the arm is at, Analog Sensors only output a voltage
     * @return The height of the arm [in voltage]
     */
    public double getArmHeight()
    {
        return ArmSensor.getVoltage();
    }
    
    /**
     * Updates the current odometry position of the robot
     */
    public void odometryUpdater()
    {
        odometry.update( Front.getCurrentPosition(),
                         Right.getCurrentPosition(),
                         Left.getCurrentPosition(),
                         Back.getCurrentPosition(), 
                         imu.getAngularOrientation().firstAngle );
        
        armHeight = ArmSensor.getVoltage();
    }
    
    /**
     * Sets all encoder values to 0
     * - Stops motors when called
     */
    public void resetEncoders()
    {
        Front.setMode( DcMotor.RunMode.RESET_ENCODERS );
        Right.setMode( DcMotor.RunMode.RESET_ENCODERS );
        Back.setMode( DcMotor.RunMode.RESET_ENCODERS );
        Left.setMode( DcMotor.RunMode.RESET_ENCODERS );
    }
}
