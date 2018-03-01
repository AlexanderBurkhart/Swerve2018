package org.usfirst.frc.team619.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Lift {
	
	TalonSRX lift1;
	TalonSRX lift2;
	TalonSRX lift3;
	
	public Lift(TalonSRX l1, TalonSRX l2, TalonSRX l3)
	{
		lift1 = l1;
		lift2 = l2;
		lift3 = l3;
	}
	
	/**
	 * Controls speed of lift motor
	 * @param speed - variable speed of motor
	 */
	public void moveLift(double speed)
	{
		lift1.set(ControlMode.PercentOutput, -speed);
		lift2.set(ControlMode.PercentOutput, speed);
		lift3.set(ControlMode.PercentOutput, speed);
	}
	
	/**
	 * Stops lift 
	 */
	public void stopLift()
	{
		lift1.set(ControlMode.PercentOutput, 0);
		lift2.set(ControlMode.PercentOutput, 0);
		lift3.set(ControlMode.PercentOutput, 0);
	}
}
