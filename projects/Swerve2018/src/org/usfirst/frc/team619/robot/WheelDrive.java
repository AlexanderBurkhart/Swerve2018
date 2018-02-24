package org.usfirst.frc.team619.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class WheelDrive {
	
	private final double MAX_VOLTS = 4.95;
	
	private TalonSRX angleMotor;
	private TalonSRX speedMotor;
	private PIDController pidController;
	public static final double p=0.3, i=0.0001, d=139;
	private double encoderUnitsPerRotation = 22235.4285714; //76/14 ratio 4096(encoder ticks per spin)
	private double targetAngle = 0;
	private double driveSpeed = 0;
	
	boolean stopRotating = false;
	
	/**
	 * wheelDrive constructor
	 * configures talons
	 * @param angleMotor - variable index of angle motor
	 * @param speedMotor - variable index of drive motor
	 */
	public WheelDrive(int angleMotor, int speedMotor)
	{
		this.angleMotor = new TalonSRX(angleMotor);
		this.speedMotor = new TalonSRX(speedMotor);
		
		this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		
		//NEVER UNCOMMENT THIS LINE UNLESS YOU WANT TO RESET ENCODER POSITION WHERE IT IS WHEN ROBOT TURNS ON
		//this.angleMotor.setSelectedSensorPosition(0, 0, 0);
		
		this.angleMotor.overrideLimitSwitchesEnable(false);
		this.angleMotor.setNeutralMode(NeutralMode.Brake);
		
		this.angleMotor.configNominalOutputForward(0, 10);
		this.angleMotor.configNominalOutputReverse(0, 10);
		this.angleMotor.configPeakOutputForward(1.0, 10);
		this.angleMotor.configPeakOutputReverse(-1.0, 10);
		
		//Limit Current
		currentLimit(this.angleMotor,25,30);
		currentLimit(this.speedMotor,35,40);
		
		//talon config
		this.angleMotor.setInverted(true); //i CHANGED THIS TO TRUE
		this.angleMotor.setSensorPhase(true);
		
		//this.angleMotor.configAllowableClosedloopError(0, 0, 10);
		
		this.angleMotor.config_kP(0, 0.6, 10);
		this.angleMotor.config_kI(0, 0.001, 10);
		this.angleMotor.config_kD(0, 10, 10);
	}
	
	/**
	 * drive the wheel with speed driveSpeed
	 */
	public void drive()
	{
		speedMotor.set(ControlMode.PercentOutput, driveSpeed);
	}
	
	/**
	 * set drive speed
	 * @param speed - drive speed value to set to
	 */
	public void setDriveSpeed(double speed) {
		driveSpeed = speed;
	}
	
	/**
	 * @return drive speed
	 */
	public double getDriveSpeed()
	{
		return driveSpeed;
	}
	
	/**
	 * stops driving the robot
	 */
	public void stop()
	{
		speedMotor.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * goes to angle set by targetAngle in getDeltaTheta()
	 */
	public void goToAngle(){
		angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition(0) + angleToEncoderUnit(getDeltaTheta()));
	}
	
	/**
	 * converts angle to encoder unit
	 * @param angle - angle to convert to
	 * @return angle in encoder units
	 */
	public int angleToEncoderUnit(double angle)
	{
		double deltaEncoder;
		deltaEncoder = angle*(encoderUnitsPerRotation/360.0);
		
		return (int)deltaEncoder;
	}
	
	/**
	 * @return get target angle of wheel
	 */
	public int getTargetAngle()
	{
		return (int)(targetAngle);
	}

	/**
	 * @return get current Angle of wheel
	 */
	public int getCurrentAngle()
	{
		return encoderUnitToAngle(angleMotor.getSelectedSensorPosition(0));
	}
	
	/**
	 * finds the closest distance to target angle
	 * @return new target angle
	 */
	public double getDeltaTheta(){
		double deltaTheta = getTargetAngle() - getCurrentAngle();
		
		while ((deltaTheta < -90) || (deltaTheta > 90)){
//			if ( label.equals("rotateMotor") )
//				System.out.println( "                          --> " + deltaTheta + "/" + speed );
			if(deltaTheta > 90){
				deltaTheta -= 180;
				driveSpeed *= -1;
			}else if(deltaTheta < -90){
				deltaTheta += 180;
				driveSpeed *= -1;
			}
		}
//		if ( label.equals("rotateMotor") )
//			System.out.println( "                          --> " + deltaTheta + "/" + speed );
		
		return deltaTheta;		
	}
	
	/**
	 * converts encoder units to angle
	 * @param e - encoder unit to convert
	 * @return e to angle
	 */
	private int encoderUnitToAngle(double e)
	{
		double angle = 0;
		if(e >= 0)
		{
			angle = (e * (360.0/encoderUnitsPerRotation));
			angle = angle % 360;
		}
		else if(e < 0) {
			angle = (e * (360.0/encoderUnitsPerRotation));
			angle = angle % 360 + 360;
		}
		return (int)angle;
	}
	
	/**
	 * sets target angle
	 * @param angle - angle to set as target angle
	 */
	public void setTargetAngle(double angle)
	{
		targetAngle = angle;
	}

	/**
	 * gets rotate talon
	 * @return rotate talon
	 */
	public TalonSRX getRotateTalon()
	{
		return angleMotor;
	}
	
	/**
	 * gets drive talon
	 * @return drive talon
	 */
	public TalonSRX getDriveTalon()
	{
		return speedMotor;
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
