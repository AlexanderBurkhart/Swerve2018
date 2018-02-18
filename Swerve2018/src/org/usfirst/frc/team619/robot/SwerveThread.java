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

public class SwerveThread extends RobotThread {
	
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
	
	public SwerveThread(int period, ThreadManager threadManager, WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, 
						WheelDrive frontLeft, int l1, int l2, int l3, int il, int ir) {
		super(period, threadManager);
		this.backRight = backRight;
		this.backLeft = backLeft;
		this.frontRight = frontRight;
		this.frontLeft = frontLeft;
		
		lift1 = new TalonSRX(l1);
		lift2 = new TalonSRX(l2);
		lift3 = new TalonSRX(l3);
		
		intakeLeft = new TalonSRX(il);
		intakeRight = new TalonSRX(ir);
		
		wheelArray[0] = frontRight;
		wheelArray[1] = frontLeft;
		wheelArray[2] = backLeft;
		wheelArray[3] = backRight;

		notMoving = true;
		
		imu = new AHRS(SPI.Port.kMXP);
		
		targetHeading = imu.getAngle();
		
		isRobotCentric = true;
		isFieldCentric = false;
		
		start();
	}
	
	public void autoZero()
	{
		/* IMPOSSIBLE TO AUTO ZERO
		 * IMPOSSIBLE BECAUSE GEAR RATIO BELOW ENCODER (76:14)
		 * SO THAT THE ENCODER DOES NOT KNOW HOW MANY SPINS IT HAS DONE, SO IT IT ALWAYS OFF BY 66 DEGREES MAX (360/(76/14))
		 */
		
	}
	
	public void moveLift(double speed)
	{
		lift1.set(ControlMode.PercentOutput, speed);
		lift2.set(ControlMode.PercentOutput, speed);
		lift3.set(ControlMode.PercentOutput, speed);
	}
	
	public void stopLift()
	{
		lift1.set(ControlMode.PercentOutput, 0);
		lift2.set(ControlMode.PercentOutput, 0);
		lift3.set(ControlMode.PercentOutput, 0);
	}
	
	public void moveIntake(double speed)
	{
		intakeRight.set(ControlMode.PercentOutput, -speed);
		intakeLeft.set(ControlMode.PercentOutput, -speed);
	}
	
	public void stopIntake()
	{
		intakeRight.set(ControlMode.PercentOutput, 0);
		intakeLeft.set(ControlMode.PercentOutput, 0);
	}
	
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
		double xAxis = deadzone(driveJoystick.getRawAxis(1));
        double yAxis = deadzone(driveJoystick.getRawAxis(0));
        double zTurn = deadzone(driveJoystick.getRawAxis(4));
		
        move(xAxis, yAxis, zTurn);
        
        if(drive.getAButton())
		{
			setRobotCentric();
		}
        else if(drive.getXButton())
        {
        	setFieldCentric();
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
//        //lift
//        if(secondary.getYButton())
//		{	
//			moveLift(1);
//		}
//        else if(secondary.getBButton())
//        {
//        	moveLift(-0.5);
//        }
//        else
//        {
//        	stopLift();
//        }
        
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

	public void move(double x1, double y1, double x2)
	{
		if(isRobotCentric){
        	drive(x1, y1, x2);
        } else if(isFieldCentric){
        	System.out.println("isFieldCentric");
            getFieldCentric(x1, y1, x2);
        }
	}
	
	public void drive(double x1, double y1, double x2) 
	{
		notMoving = x1 == 0 && y1 == 0 && x2 == 0;
		
		if(!notMoving)
		{
			double r = Math.sqrt((L * L) + (W * W));
			
			double a = x1 - x2 * (L/r);
			double b = x1 + x2 * (L/r);
			double c = y1 - x2 * (W/r);
			double d = y1 + x2 * (W/r);	
			
	        double[] angles = new double[]{ atan2(b,c)*180/PI,
                    atan2(b,d)*180/PI,
                    atan2(a,d)*180/PI,
                    atan2(a,c)*180/PI };
 
	        double[] speeds = new double[]{ sqrt(b*b+c*c),
                    sqrt(b*b+d*d),
                    sqrt(a*a+d*d),
                    sqrt(a*a+c*c) };
		
	        double max = speeds[0];
	        if ( speeds[1] > max ) max = speeds[1];
	        if ( speeds[2] > max ) max = speeds[2];
	        if ( speeds[3] > max ) max = speeds[3];
	        
	        if ( max > 1 ) {
	            speeds[0] /= max;
	            speeds[1] /= max;
	            speeds[2] /= max;
	            speeds[3] /= max;
	        }
			
			for( int i=0; i < wheelArray.length; i++ ) {
				
				wheelArray[i].setDriveSpeed(speeds[i]);
	            wheelArray[i].setTargetAngle(angles[i]);
	        }
			
//			if(Math.abs(x2) > 0)
//			{
//				wheelArray[1].setDriveSpeed(-speeds[1]);
//				wheelArray[3].setDriveSpeed(-speeds[3]);
//			}
//	
	        for(int i = 0; i < wheelArray.length; i++)
	        {
	        	wheelArray[i].goToAngle();
	            wheelArray[i].drive();
	        }
		}
		else
		{
			for(int i = 0; i < wheelArray.length; i++)
	        {
	        	wheelArray[i].stop();
	        }
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
	
    public void getFieldCentric( double x1, double y1, double x2 ) {
    	System.out.println("I'm in");
        // correspondence to paper http://www.chiefdelphi.com/media/papers/download/3028
        //     RY  <=>   FWD
        //     RX  <=>   STR
        //     LX  <=>   RCW

        //  imu.getYaw( ) returns angle between -180 and 180
        double theta = imu.getYaw( );
        //System.out.println("Theta: " + theta);
        while ( theta < 0 ) theta += 360;
        if(x2 != 0)
        	targetHeading = theta;
        theta = toRadians(theta);
        double temp = y1*cos(theta) + x1*sin(theta);
        x1 = -y1*sin(theta) + x1*cos(theta);
        y1 = temp;

        drive(x1, y1, x2);
    }

	
	private double deadzone(double val) {
		if(Math.abs(val) < 0.06)
			return 0;
		return val;
	}
	
    public void delay(int milliseconds){
    	try {
    		Thread.sleep(milliseconds);
    	} catch(InterruptedException e) {
    		Thread.currentThread().interrupt();
    	}
    }
}
