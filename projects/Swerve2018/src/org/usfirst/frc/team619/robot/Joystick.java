package org.usfirst.frc.team619.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * @author carobotics
 */

public class Joystick {
    
    public static class Button{
        public static final int TRIGGER = 0;
        
        public static final int BUTTON1 = 1;
        public static final int BUTTON2 = 2;
        public static final int BUTTON3 = 3;
        public static final int BUTTON4 = 4;
        public static final int BUTTON5 = 5;
        public static final int BUTTON6 = 6;
        public static final int BUTTON7 = 7;
        public static final int BUTTON8 = 8;
        public static final int BUTTON9 = 9;
        public static final int BUTTON10 = 10;
        public static final int BUTTON11 = 11;
        public static final int BUTTON12 = 12;
        
        public static final int BUMPER = 13;
        public static final int TOP = 14;
    }
    
    public static class Axis{
        public static final int AXIS_X = 0;
        public static final int AXIS_Y = 1;
        public static final int AXIS_Z = 2;
        public static final int AXIS_TWIST = 3;
        public static final int STICK_MAGNITUDE = 4;
        public static final int AXIS_THROTTLE = AXIS_Z+1; // This is the correct mapping on Attack 3's and the Saitek joysticks. Might want to change this for other joysticks
        
        public static final int MAX_AXIS_VALUE = AXIS_THROTTLE + 1;
    }
    
    protected edu.wpi.first.wpilibj.Joystick joystick;
    
    private boolean reversedAxis[];
    
    private double deadband = 0.05;
    
    public Joystick(){}
    
    public Joystick(int id){
        joystick = new edu.wpi.first.wpilibj.Joystick(id);
        joystick.setAxisChannel(edu.wpi.first.wpilibj.Joystick.AxisType.kTwist, 3);
        reversedAxis = new boolean[Axis.MAX_AXIS_VALUE];
    }
    
    public Joystick(int id, double deadband){
        this(id);
        this.deadband = Math.abs(deadband);
    }
    
    public boolean getButton(int button){
        switch(button){
            case Button.TRIGGER:
                return joystick.getTrigger();
            case Button.BUTTON1:
                return joystick.getRawButton(1);
            case Button.BUTTON2:
                return joystick.getRawButton(2);
            case Button.BUTTON3:
                return joystick.getRawButton(3);
            case Button.BUTTON4:
                return joystick.getRawButton(4);
            case Button.BUTTON5:
                return joystick.getRawButton(5);
            case Button.BUTTON6:
                return joystick.getRawButton(6);
            case Button.BUTTON7:
                return joystick.getRawButton(7);
            case Button.BUTTON8:
                return joystick.getRawButton(8);
            case Button.BUTTON9:
                return joystick.getRawButton(9);
            case Button.BUTTON10:
                return joystick.getRawButton(10);
            case Button.BUTTON11:
                return joystick.getRawButton(11);
            case Button.BUTTON12:
                return joystick.getRawButton(12);
            case Button.BUMPER:
                return joystick.getRawButton(13);
            case Button.TOP:
                return joystick.getTop();
            default:
                System.out.println("Invalid button ID: "+button);
                return false;
        }
    }
    
    public double getAxis(int axis){
        return getAxis(axis, false);
    }
    
    
    public double getAxis(int axis, boolean overrideReverse){
        //System.out.println("RAXIS= "+reversedAxis.length);
        //System.out.println("AXIS = "+axis);
        if(!overrideReverse && reversedAxis[axis]){
            return -1.0 * getAxis(axis, true);
        }
        
        double val;
        
        switch(axis){
            case Axis.AXIS_X:
                val = joystick.getX();
                break;
            case Axis.AXIS_Y:
                val = joystick.getY();
                break;
            case Axis.AXIS_Z:
                val = joystick.getZ(); // On both the Attack 3 and the Saitek, the throttle is mappe to the z axis and the twist axis. No clue what is mapped to the throttle axis
                val *= -1.0; // Because this is the throttle axis, make it behave like it's labelled and expected
                val += 1.0; // i.e. bottom is 0 and top is 1.0
                val /= 2.0;
                break;
            case Axis.AXIS_TWIST:
                val = joystick.getTwist();
                break;
            case Axis.STICK_MAGNITUDE:
                val = joystick.getMagnitude();
                break;
            default:
                return 0;
        }
        
        if(deadband > Math.abs(val)) val = 0;
        
        return val;
    }
    
    public void setReversed(int axis, boolean isReversed){
        reversedAxis[axis] = isReversed;
    }
    
    public void debugPrint(){
        System.out.print("Y: " + getAxis(Joystick.Axis.AXIS_Y) + " Throttle: " + getAxis(Joystick.Axis.AXIS_THROTTLE) + " Buttons: ");
        if(getButton(Button.BUMPER)) System.out.print("BUMPER ");
        if(getButton(Button.TOP)) System.out.print("TOP ");
        if(getButton(Button.TRIGGER)) System.out.print("TRIGGER ");
        
        if(getButton(Button.BUTTON1)) System.out.print("BUTTON1 ");
        if(getButton(Button.BUTTON2)) System.out.print("BUTTON2 ");
        if(getButton(Button.BUTTON3)) System.out.print("BUTTON3 ");
        if(getButton(Button.BUTTON4)) System.out.print("BUTTON4 ");
        if(getButton(Button.BUTTON5)) System.out.print("BUTTON5 ");
        if(getButton(Button.BUTTON6)) System.out.print("BUTTON6 ");
        if(getButton(Button.BUTTON7)) System.out.print("BUTTON7 ");
        if(getButton(Button.BUTTON8)) System.out.print("BUTTON8 ");
        if(getButton(Button.BUTTON9)) System.out.print("BUTTON9 ");
        if(getButton(Button.BUTTON10)) System.out.print("BUTTON10 ");
        if(getButton(Button.BUTTON11)) System.out.print("BUTTON11 ");
        if(getButton(Button.BUTTON12)) System.out.print("BUTTON12 ");
        
        System.out.println();
        
    }
}
