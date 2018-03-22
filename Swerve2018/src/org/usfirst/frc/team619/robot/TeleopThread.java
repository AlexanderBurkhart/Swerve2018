package org.usfirst.frc.team619.robot;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

//import org.usfirst.frc.team619.subsystems.GripPipeline;

public class TeleopThread extends RobotThread {
	
	int currentAngle, targetAngle;
	WheelDrive[] wheelArray = new WheelDrive[4];
	double[] speeds = new double[4];
	
	double steer_value;
	
	public final double L = 6.35;
	public final double W = 9.525;
	
	double targetHeading;
	
	private WheelDrive backRight;
	private WheelDrive backLeft;
	private WheelDrive frontRight;
	private WheelDrive frontLeft;
	
	//drive
	private XboxController drive = new XboxController(0);
	private Joystick driveJoystick = new Joystick(0);
	
	//secondary
	private XboxController secondary = new XboxController(1);
	private Joystick secondaryJoystick = new Joystick(1);
	
	boolean notMoving;
	
	//Manipulators
	TalonSRX lift1;
	TalonSRX lift2;
	TalonSRX lift3;
	
	TalonSRX intakeLeft;
	TalonSRX intakeRight;
	
	//NAVX
	AHRS imu;
	
	//modes
	boolean isRobotCentric;
	boolean isFieldCentric;
	
	//PID VARS
	//pid values
	double kP;
	double kI;
	double kD;
	
	double theta;
	double mag;
	
	double speed;
	
	SwerveDriveBase driveBase;
	int scalePercent;
	
	boolean alreadyPressed;
	
	//Manipulators
	Lift lift;
	Intake intake;
	Ramp ramp;
	
	/**
	 * Constructor for teleop thread
	 * @param period - threadManager variable
	 * @param threadManager - threadManager object
	 * 
	 * @param backRight - wheelDrive object for back right
	 * @param backLeft - wheelDrive object for back left
	 * @param frontRight - wheelDrive object for front right
	 * @param frontLeft - wheelDrive object for front left
	 * 
	 * @param l1 - variable index for lift 1
	 * @param l2 - variable index for lift 2
	 * @param l3 - variable index for lift 3
	 * 
	 * @param il - variable index for left intake
	 * @param ir - variable index for right intake
	 * 
	 * @param iSwitch LimitSwitch object for limit switch
	 */	
	public TeleopThread(int period, ThreadManager threadManager, WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, 
						WheelDrive frontLeft, Lift l, Intake i, Ramp r) {
		super(period, threadManager);
		this.backRight = backRight;
		this.backLeft = backLeft;
		this.frontRight = frontRight;
		this.frontLeft = frontLeft;
		
		lift = l;
		intake = i;
		ramp = r;
		
		wheelArray[0] = frontRight;
		wheelArray[1] = frontLeft;
		wheelArray[2] = backLeft;
		wheelArray[3] = backRight;

		notMoving = true;
		
		imu = new AHRS(SPI.Port.kMXP);
		
		alreadyPressed = false;
		
		targetHeading = imu.getAngle();
		
		isRobotCentric = true;
		isFieldCentric = false;
		
		scalePercent = 10;
		
		driveBase = new SwerveDriveBase(backRight, backLeft, frontRight, frontLeft);
		
		start();
	}
	
	public void autoZero()
	{
		/* IMPOSSIBLE TO AUTO ZERO
		 * IMPOSSIBLE BECAUSE GEAR RATIO BELOW ENCODER (76:14)
		 * SO THAT THE ENCODER DOES NOT KNOW HOW MANY SPINS IT HAS DONE, SO IT IT ALWAYS OFF BY 66 DEGREES MAX (360/(76/14))
		 */
		
	}
	
	/**
	 * refreshes inputs continuously
	 */
	protected void cycle() {
		/*
		 * Driver Controller
		 * Functions:
		 * Right Joystick - Turn
		 * Left Joystick - Drive
		 * X Button - Field Centric
		 * A Button - Robot Centric
		 * Right Bumper - 20% Speed
		 * Dpad (Up/Down) - Speed Change (10% or -10%)
		 * Trigger - Break
		 * Both Bumpers - Turbo (100% Speed)
		 */
		double xAxis = deadzone(drive.getX(Hand.kRight));
        double yAxis = deadzone(drive.getY(Hand.kRight));
        double zTurn = deadzoneRotate(drive.getX(Hand.kLeft));
        
//        System.out.println("xAxis: " + xAxis);
//        System.out.println("yAxis: " + yAxis);
//        System.out.println("zTurn: " + zTurn);
//        System.out.println();
        
//        System.out.println(imu.getYaw());
        
        if(drive.getAButton())
		{
			setRobotCentric();
		}
        else if(drive.getXButton())
        {
        	setFieldCentric();
        }
        else if(drive.getYButton())
        {
        	imu.zeroYaw();
        }
        else if(drive.getBumper(Hand.kRight))
        {
        	xAxis *= 0.3;
        	yAxis *= 0.3;
        	zTurn *= 0.5;
        }
        else if(drive.getPOV() == 0)
        {
        	if(!(scalePercent == 10) && !alreadyPressed)
        	{
        		scalePercent += 1;
        		alreadyPressed = true;
        	}
        }
        else if(drive.getPOV() == 180)
        {
        	if(!(scalePercent == 1) && !alreadyPressed)
        	{
        		scalePercent -= 1;
        		alreadyPressed = true;
        	}
        }
        else
        {
        	alreadyPressed = false;
        }
        System.out.println("scalePercent: " + scalePercent);
        
        move(xAxis * scalePercent*0.1, yAxis * scalePercent*0.1, zTurn*0.7*scalePercent*0.1);
        
        /*
         * Secondary Controller
         * Functions:
         * Left Joystick - Manual Lift
         * Dpad (Up/Down) - Auto Lift
         * B - Intake In
         * Y - Intake Out
         * A - Ramp Release
         */
        //lift
        if(secondary.getPOV() == 0)
		{	
			lift.moveLift(1);
		}
        else if(secondary.getPOV() == 180)
        {
        	lift.moveLift(-0.5);
        }
        else
        {
        	lift.stopLift();
        }
        
        //intake
        if(secondary.getBButton())
        {
        	//intake
        	intake.moveIntake(1);
        }
        else if(secondary.getYButton())
        {
        	//outake
        	intake.moveIntake(-1);
        }
        else if(secondary.getBumper(Hand.kRight))
        {
        	intake.intakeRight.set(ControlMode.PercentOutput, 1);
        }
        else if(secondary.getBumper(Hand.kLeft))
        {
        	intake.intakeLeft.set(ControlMode.PercentOutput, 1);
        }
        else if(secondary.getTriggerAxis(Hand.kRight) > 0)
        {
        	intake.intakeRight.set(ControlMode.PercentOutput, -1);
        }
        else if(secondary.getTriggerAxis(Hand.kLeft) > 0)
        {
        	intake.intakeLeft.set(ControlMode.PercentOutput, -1);
        }
        else if(secondary.getXButton())
        {
        	//ramp windup
        	ramp.windUp();
        }
        else if(secondary.getAButton())
        {
        	//ramp winddown
        	ramp.windDown();
        }
        else
        {
        	//ramp.stop();
        	intake.stopIntake();
        }
	}
	
	/**
	 * calls driveBase to move swerve modules
	 * @param x1 - double x axis of right joystick
	 * @param y1 - double y axis of right joystick
	 * @param x2 - double x axis of left joystick
	 */
	public void move(double x1, double y1, double x2)
	{
		if(isRobotCentric){
        	driveBase.drive(x1, -y1, x2);
        } else if(isFieldCentric){
        	//System.out.println("isFieldCentric");
            driveBase.getFieldCentric(x1, -y1, x2);
        }
	}
	
	//states
	public void setFieldCentric()
	{
		isFieldCentric = true;
		isRobotCentric = false;
	}
	
	public void setRobotCentric()
	{
		isFieldCentric = false;
		isRobotCentric = true;
	}
	
	/**
	 * sets val below 0.06 to zero
	 * @param val - variable input
	 * @return
	 */
	private double deadzone(double val) {
		if(Math.abs(val) < 0.06)
			return 0;
		return val;
	}
	
	/**
	 * sets val below 0.06 to zero
	 * @param val - variable input
	 * @return
	 */
	private double deadzoneRotate(double val) {
		if(Math.abs(val) < 0.25)
			return 0;
		return val;
	}
	
	/**
	 * Delays thread in milliseconds
	 * @param milliseconds - variable time to delay in ms 
	 */
    public void delay(int milliseconds){
    	try {
    		Thread.sleep(milliseconds);
    	} catch(InterruptedException e) {
    		Thread.currentThread().interrupt();
    	}
    }
    
	/**
	 * Limits current to selected motor
	 * @param talon - TalonSRX object
	 * @param continousLimit - variable limit of current
	 * @param currentLimit - variable limit of peak
	 */
	public void currentLimit(TalonSRX talon, int continuousLimit, int currentLimit) {
		talon.configContinuousCurrentLimit(continuousLimit, 0);
		talon.configPeakCurrentLimit(currentLimit, 0);
		talon.configPeakCurrentDuration(100, 0);
		talon.enableCurrentLimit(true);

	}
}
