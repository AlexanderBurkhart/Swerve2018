package org.usfirst.frc.team619.robot;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class SwerveDriveBase {
	
	boolean notMoving;
	
	public final double L = 6.35;
	public final double W = 9.525;
	
	private WheelDrive backRight;
	private WheelDrive backLeft;
	private WheelDrive frontRight;
	private WheelDrive frontLeft;
	
	WheelDrive[] wheelArray = new WheelDrive[4];
	double[] speeds = new double[4];
	
	//modes
	boolean isRobotCentric;
	boolean isFieldCentric;
	
	//NAVX
	AHRS imu;
	double targetHeading;
	
	/**
	 * Constructor
	 * @param backRight - back right wheel of robot
	 * @param backLeft - back left wheel of robot
	 * @param frontRight - front right wheel of robot
	 * @param frontLeft - front left wheel of robot
	 */
	public SwerveDriveBase(WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft)
	{
		this.backRight = backRight;
		this.backLeft = backLeft;
		this.frontRight = frontRight;
		this.frontLeft = frontLeft;
		
		wheelArray[0] = frontRight;
		wheelArray[1] = frontLeft;
		wheelArray[2] = backLeft;
		wheelArray[3] = backRight;
		
		notMoving = true;
		
		isRobotCentric = true;
		isFieldCentric = false;
		
		imu = new AHRS(SPI.Port.kMXP);
		targetHeading = imu.getAngle();
	}
	
	/**
	 * drives the robot
	 * @param x1 - x axis of robot (right joystick)
	 * @param y1 - y axis of robot (right joystick)
	 * @param x2 - x axis rotation value of robot (left joystick)
	 */
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
	            speeds[0] = max; //was dividing
	            speeds[1] = max;
	            speeds[2] = max;
	            speeds[3] = max;
	        }
			
	        //drive speed
	        wheelArray[0].setDriveSpeed(speeds[0]);
			wheelArray[2].setDriveSpeed(speeds[2]);
			if(Math.abs(x2) > 0)
			{
				wheelArray[1].setDriveSpeed(-speeds[1]);
				wheelArray[3].setDriveSpeed(-speeds[3]);
			}
			else
			{
				wheelArray[1].setDriveSpeed(speeds[1]);
				wheelArray[3].setDriveSpeed(speeds[3]);
			}
	        
			for( int i=0; i < wheelArray.length; i++ ) {	
	            wheelArray[i].setTargetAngle(angles[i]);
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
	
	/**
	 * sets field centric
	 */
	public void setFieldCentric()
	{
		isFieldCentric = true;
		isRobotCentric = false;
	}
	
	/**
	 * sets robot centric
	 */
	public void setRobotCentric()
	{
		isFieldCentric = false;
		isRobotCentric = true;
	}
	
	/**
	 * gets field centric variables for drive
	 * @param x1 - x axis (left joystick)
	 * @param y1 - y axis (left joystick)
	 * @param x2 - z axis (right joystick)
	 */
    public void getFieldCentric( double x1, double y1, double x2 ) {
    	//System.out.println("I'm in");
        // correspondence to paper http://www.chiefdelphi.com/media/papers/download/3028
        //     RY  <=>   FWD
        //     RX  <=>   STR
        //     LX  <=>   RCW

        //  imu.getYaw( ) returns angle between -180 and 180
        double theta = imu.getYaw( );
        System.out.println("theta: " + theta);
        //System.out.println("Theta: " + theta);
        while ( theta < 0 ) theta += 360; //COMMENT THIS OUT AGAIN IF IT SCREWS UP
        if(x2 != 0)
        	targetHeading = theta;
        theta = -toRadians(theta); //CHANGE THIS TO NEGATIVE IF IT SCEWS UP
        double temp = y1*cos(theta) + x1*sin(theta);
        x1 = -y1*sin(theta) + x1*cos(theta);
        y1 = temp;

        drive(x1, y1, x2);
    }
}
