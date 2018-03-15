package org.usfirst.frc.team619.robot;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveBase {
	
	boolean notMoving;
	
	public final double L = 16.129;
	public final double W = 24.194;
	
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
		double r = Math.sqrt((L * L) + (W * W));
		
		double multi = Math.sqrt(2)/2;
		
		double[] fr = new double[] {x1 + multi*x2, y1 + -multi*x2, 0};
		fr[2] = Math.sqrt(Math.pow(fr[0], 2) + Math.pow(fr[1], 2));
		
		double[] fl = new double[] {x1 + multi*x2, y1 + multi*x2, 0};
		fl[2] = Math.sqrt(Math.pow(fl[0], 2) + Math.pow(fl[1], 2));
		
		double[] bl = new double[] {x1 + -multi*x2, y1 + multi*x2, 0};
		bl[2] = Math.sqrt(Math.pow(bl[0], 2) + Math.pow(bl[1], 2));
		
		double[] br = new double[] {x1 + -multi*x2, y1 + -multi*x2, 0};
		br[2] = Math.sqrt(Math.pow(br[0], 2) + Math.pow(br[1], 2));
		
		double[][] valWheel = new double[][]{fr, fl, bl, br};
		
		double[] angles = new double[] {atan2(valWheel[0][1], valWheel[0][0])*180/PI,
				atan2(valWheel[1][1], valWheel[1][0])*180/PI,
				atan2(valWheel[2][1], valWheel[2][0])*180/PI,
				atan2(valWheel[3][1], valWheel[3][0])*180/PI
		};
		
		double[] speeds = new double[]{valWheel[0][2],
				valWheel[1][2],
				valWheel[2][2],
				valWheel[3][2]
		};
		
		if( abs(y1) < 0.05 && abs(x1) < 0.05 && abs(x2) < 0.05){
			for(WheelDrive wheel : wheelArray) {
				 wheel.setDriveSpeed(0);
			 }
       } else {
           //Set target angle
           for( int i=0; i < wheelArray.length; ++i ) {
               wheelArray[i].setTargetAngle(angles[i]);
               wheelArray[i].setDriveSpeed(speeds[i]);
           }
       }
	
	  for(WheelDrive wheel : wheelArray)
          wheel.goToAngle();
      for(WheelDrive wheel : wheelArray)
          wheel.drive();		
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
        //System.out.println("theta: " + theta);
        //System.out.println("Theta: " + theta);
        while ( theta < 0 ) theta += 360; //COMMENT THIS OUT AGAIN IF IT SCREWS UP
        if(x2 != 0)
        	targetHeading = theta;
        theta = toRadians(theta); //CHANGE THIS TO NEGATIVE IF IT SCEWS UP
        double temp = y1*cos(theta) + x1*sin(theta);
        x1 = -y1*sin(theta) + x1*cos(theta);
        y1 = temp;

        drive(x1, y1, x2);
    }
}
