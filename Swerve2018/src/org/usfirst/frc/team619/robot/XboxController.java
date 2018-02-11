package org.usfirst.frc.team619.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class XboxController {

	protected edu.wpi.first.wpilibj.XboxController controller;
	/**
	 * Currently just a wrapper class. Adds nothing new to XboxController from wpilib
	 * This class exists in case we want to add something extra
	 */
	public XboxController(int port) {
		controller = new edu.wpi.first.wpilibj.XboxController(port);
	}
	
	/**
	* Get the X axis value of the controller.
	*
	* @param hand Side of controller whose value should be returned.
	* @return The X axis value of the controller.
	*/
	public double getX(Hand hand) {
		return controller.getX(hand);
	}
	
	/**
	 * Get the Y axis value of the controller.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return The Y axis value of the controller.
	 */
	public double getY(Hand hand) {
		return controller.getY(hand);
	}
	
	/**
	 * Read the value of the bumper button on the controller.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return The state of the button.
	 */
	public boolean getBumper(Hand hand) {
		return controller.getBumper(hand);
	}
	
	/**
	 * Get the trigger axis value of the controller.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return The trigger axis value of the controller.
	 */
	public double getTriggerAxis(Hand hand) {
		return controller.getTriggerAxis(hand);
	}
	
	/**
	 * Read the value of the A button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getAButton() {
		return controller.getAButton();
	}
	
	/**
	 * Read the value of the B button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getBButton() {
		return controller.getBButton();
	}
	
	/**
	 * Read the value of the X button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getXButton() {
		return controller.getXButton();
	}
	
	/**
	 * Read the value of the Y button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getYButton() {
		return controller.getYButton();
	}
	
	/**
	 * Read the value of the stick button on the controller.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return The state of the button.
	 */
	public boolean getStickButton(Hand hand) {
		return controller.getStickButton(hand);
	}
	
	/**
	 * Read the value of the back button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getBackButton() {
		return controller.getBackButton();
	}
	
	/**
	 * Read the value of the start button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getStartButton() {
		return controller.getStartButton();
	}
	
	public int getPOV() {
		return controller.getPOV(0);
	}
}
