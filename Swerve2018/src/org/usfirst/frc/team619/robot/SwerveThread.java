package org.usfirst.frc.team619.robot;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
	
	private XboxController xbox = new XboxController(0);
	private Joystick joystick = new Joystick(0);
	
	boolean notMoving;
	
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
	
	public SwerveThread(int period, ThreadManager threadManager, WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
		super(period, threadManager);
		this.backRight = backRight;
		this.backLeft = backLeft;
		this.frontRight = frontRight;
		this.frontLeft = frontLeft;
		
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
		//frontright (0)
		wheelArray[0].setTargetAngle(31);
		wheelArray[0].goToAngle();
		delay(2000);
		//wheelArray[0].getRotateTalon().setSelectedSensorPosition(0, 0, 0);
		
		//frontleft (1)
		wheelArray[1].setTargetAngle(58);
		wheelArray[1].goToAngle();
		delay(2000);
		//wheelArray[1].getRotateTalon().setSelectedSensorPosition(0, 0, 0);
		
		//backLeft (2)
		wheelArray[2].setTargetAngle(55);
		wheelArray[2].goToAngle();
		delay(2000);
		//wheelArray[2].getRotateTalon().setSelectedSensorPosition(0, 0, 0);
		
		//backRight (3)
		wheelArray[3].setTargetAngle(27);
		wheelArray[3].goToAngle();
		delay(2000);
		//wheelArray[3].getRotateTalon().setSelectedSensorPosition(0, 0, 0);
		

	}
	
	protected void cycle() {
		double xAxis = deadzone(joystick.getRawAxis(1));
        double yAxis = deadzone(joystick.getRawAxis(0));
        double zTurn = deadzone(joystick.getRawAxis(4));
		
        move(xAxis, yAxis, zTurn);
        
        if(xbox.getYButton())
		{	
			System.out.println("ZERO");
			autoZero();
		}
        else if(xbox.getAButton())
		{
			setRobotCentric();
		}
        else if(xbox.getXButton())
        {
        	setFieldCentric();
        }
        
        delay(50);
	}

	public void move(double x1, double y1, double x2)
	{
		if(isRobotCentric){
        	drive(x1, y1, x2);
        } else if(isFieldCentric){
        	System.out.println("isFieldCentric");
            getFieldCentric(-x1, y1, x2);
        }
	}
	
	public void drive(double x1, double y1, double x2) 
	{
		notMoving = x1 == 0 && y1 == 0 && x2 == 0;
		
		if(!notMoving)
		{
			double r = Math.sqrt((L * L) + (W * W));
			x1 *= -1;
			
			double a = x1 - x2 * (L/r);
			double b = x1 + x2 * (L/r);
			double c = y1 - x2 * (W/r);
			double d = y1 + x2 * (W/r);
			
//			double backRightSpeed = Math.sqrt((a*a) + (d*d));
//			double backLeftSpeed = Math.sqrt((a*a) + (c*c));
//			double frontRightSpeed = Math.sqrt((b*b) + (d*d));
//			double frontLeftSpeed = Math.sqrt((b*b) + (c*c));
//			
//			System.out.println("backRightSpeed: " + backRightSpeed);
//			
			
	        double[] angles = new double[]{ atan2(b,c)*180/PI,
                    atan2(b,d)*180/PI,
                    atan2(a,d)*180/PI,
                    atan2(a,c)*180/PI };

	        double[] speeds = new double[]{ sqrt(b*b+d*d),
                    sqrt(b*b+c*c),
                    sqrt(a*a+c*c),
                    sqrt(a*a+d*d) };
//			
//			double[] speeds = {frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed};	
			
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
			
//			double backRightAngle = ((Math.atan2(a, d) / Math.PI * 180) *-1) + 180;
//			double backLeftAngle = ((Math.atan2(a, c) / Math.PI * 180) * -1) + 180;
//			double frontRightAngle = ((Math.atan2(b, d) / Math.PI * 180) * -1) + 180;
//			double frontLeftAngle = ((Math.atan2(b, c) / Math.PI * 180) * -1) + 180;
//			
//	        double backRightAngle = ((Math.atan2(a, d) / Math.PI * 180));
//			double backLeftAngle = ((Math.atan2(a, c) / Math.PI * 180));
//			double frontRightAngle = ((Math.atan2(b, d) / Math.PI * 180));
//			double frontLeftAngle = ((Math.atan2(b, c) / Math.PI * 180));
//			
//			double[] angles = {(int)frontLeftAngle, (int)backLeftAngle, (int)frontRightAngle, (int)backRightAngle};

			for( int i=0; i < wheelArray.length; i++ ) {
				
				wheelArray[i].setDriveSpeed(speeds[i]);
	            wheelArray[i].setTargetAngle(angles[i]);
	        }
			
			if(Math.abs(x2) > 0)
			{
				wheelArray[1].setDriveSpeed(-speeds[1]);
				wheelArray[3].setDriveSpeed(-speeds[3]);
			}
			
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
