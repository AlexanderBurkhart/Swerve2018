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

	
	double steer_value;

	//drive
	private XboxController drive = new XboxController(0);
	
	//secondary
	private XboxController secondary = new XboxController(1);

	//Manipulators
	TalonSRX lift1;
	TalonSRX lift2;
	TalonSRX lift3;
	
	TalonSRX intakeLeft;
	TalonSRX intakeRight;
	

	
	LimitSwitch intakeSwitch;
	
	SwerveDriveBase driveBase;

	
	//PID VARS
	//pid values
	double kP;
	double kI;
	double kD;
	
	double theta;
	double mag;
	
	double speed;
	
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
						WheelDrive frontLeft, int l1, int l2, int l3, int il, int ir, LimitSwitch iSwitch) {
		super(period, threadManager);
		
		driveBase = new SwerveDriveBase(backRight, backLeft, frontRight, frontLeft);
		
//		lift1 = new TalonSRX(l1);
//		lift2 = new TalonSRX(l2);
//		lift3 = new TalonSRX(l3);
		
//		currentLimit(lift1,35,40);
//		currentLimit(lift2,35,40);
//		currentLimit(lift3,35,40);

		intakeSwitch = iSwitch;
		
//		intakeLeft = new TalonSRX(il);
//		intakeRight = new TalonSRX(ir);
		
//		currentLimit(intakeLeft,25,30);
//		currentLimit(intakeRight,25,30);

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
	 * Limits current to selected motor
	 * @param talon - TalonSRX object
	 * @param continousLimit - variable limit of current
	 * @param currentLimit - variable limit of peak
	 */
	public void currentLimit(TalonSRX talon, int continuousLimit, int currentLimit) {
		talon.configContinuousCurrentLimit(25, 0);
		talon.configPeakCurrentLimit(30, 0);
		talon.configPeakCurrentDuration(100, 0);
		talon.enableCurrentLimit(true);

	}
	
	/**
	 * Controls speed of lift motor
	 * @param speed - variable speed of motor
	 */
	public void moveLift(double speed)
	{
//		lift1.set(ControlMode.PercentOutput, -speed);
//		lift2.set(ControlMode.PercentOutput, speed);
//		lift3.set(ControlMode.PercentOutput, speed);
	}
	
	/**
	 * Stops lift 
	 */
	public void stopLift()
	{
//		lift1.set(ControlMode.PercentOutput, 0);
//		lift2.set(ControlMode.PercentOutput, 0);
//		lift3.set(ControlMode.PercentOutput, 0);
	}
	
	/**
	 * Controls speed and direction of intake motors
	 * @param speed
	 */
	public void moveIntake(double speed)
	{
		//- for intakeright is in
		//+ for intakeleft is in
		
//		System.out.println(intakeSwitch.get());
//		if(intakeSwitch.get() == true)
//		{
//			intakeRight.set(ControlMode.PercentOutput, speed/2);	
//		}
//		else
//		{
//			intakeRight.set(ControlMode.PercentOutput, -speed);
//		}
//		intakeLeft.set(ControlMode.PercentOutput, speed);
	}
	/**
	 * Stops intake
	 */
	public void stopIntake()
	{
//		intakeRight.set(ControlMode.PercentOutput, 0);
//		intakeLeft.set(ControlMode.PercentOutput, 0);
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
        double zTurn = deadzone(drive.getX(Hand.kLeft));
		
        move(xAxis, yAxis, zTurn);
        
        if(drive.getAButton())
		{
			driveBase.setRobotCentric();
		}
        else if(drive.getXButton())
        {
        	driveBase.setFieldCentric();
        }
        
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
			moveLift(1);
		}
        else if(secondary.getPOV() == 180)
        {
        	moveLift(-1);
        }
        else
        {
        	stopLift();
        }
        
        //intake
        if(secondary.getBButton())
        {
        	//intake
        	moveIntake(0.5);
        }
        else if(secondary.getYButton())
        {
        	//outake
        	moveIntake(-0.5);
        }
        else
        {
        	stopIntake();
        }

        delay(50);
	}
/**
 * calls driveBase to move swerve modules
 * @param x1 - double x axis of right joystick
 * @param y1 - double y axis of right joystick
 * @param x2 - double x axis of left joystick
 */
	public void move(double x1, double y1, double x2)
	{
		if(driveBase.isRobotCentric){
        	driveBase.drive(-x1, y1, -x2);
        } else if(driveBase.isFieldCentric){
        	System.out.println("isFieldCentric");
            driveBase.getFieldCentric(x1, y1, x2);
        }
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
}
