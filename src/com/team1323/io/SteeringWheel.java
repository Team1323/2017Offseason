package com.team1323.io;

import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class SteeringWheel extends Joystick{
	private static final double DEAD_BAND = 0.1;
    private boolean rumbling = false;
    public ButtonCheck aButton;
    public ButtonCheck bButton;
    public ButtonCheck xButton;
    public ButtonCheck yButton;
    public ButtonCheck startButton;
    public ButtonCheck backButton;
    public ButtonCheck leftBumper;
    public ButtonCheck rightBumper;
    
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;
    
    public SteeringWheel(int usb){
    	super(usb);
    	aButton = new ButtonCheck(A_BUTTON);
        bButton = new ButtonCheck(B_BUTTON);
        xButton = new ButtonCheck(X_BUTTON);
        yButton = new ButtonCheck(Y_BUTTON);
        startButton = new ButtonCheck(START_BUTTON);
        backButton = new ButtonCheck(BACK_BUTTON);
        leftBumper = new ButtonCheck(LEFT_BUMPER);
        rightBumper = new ButtonCheck(RIGHT_BUMPER);
    }
    
    public double getWheelTurn(){
    	return Util.deadBand(getRawAxis(0), DEAD_BAND);
    }
    
    public void rumble(double rumblesPerSecond, double numberOfSeconds){
    	if(!rumbling){
    		RumbleThread r = new RumbleThread(rumblesPerSecond, numberOfSeconds);
    		r.start();
    	}
    }
    public boolean isRumbling(){
    	return rumbling;
    }
    public class RumbleThread extends Thread{
    	public double rumblesPerSec = 1;
    	public long interval = 500;
    	public double seconds = 1;
    	public double startTime = 0;
    	public RumbleThread(double rumblesPerSecond, double numberOfSeconds){
    		rumblesPerSec = rumblesPerSecond;
    		seconds = numberOfSeconds;
    		interval =(long) (1/(rumblesPerSec*2)*1000);
    	}
    	public void run(){
    		rumbling = true;
    		startTime = Timer.getFPGATimestamp();
    		try{
    			while((Timer.getFPGATimestamp() - startTime) < seconds){
		    		setRumble(RumbleType.kLeftRumble, 1);
		    		setRumble(RumbleType.kRightRumble, 1);
		    		sleep(interval);
		    		setRumble(RumbleType.kLeftRumble, 0);
		    		setRumble(RumbleType.kRightRumble, 0);
		    		sleep(interval);
    			}
    		}catch (InterruptedException e) {
				rumbling = false;
				e.printStackTrace();
			}
    		rumbling = false;
    	}
    }
    
    public class ButtonCheck{
    	boolean buttonCheck = false;
    	boolean buttonActive = false;
    	boolean longPressActive = false;
    	boolean hasBeenPressed = false;
    	private double buttonStartTime = 0;
    	private int buttonNumber;
    	
    	public ButtonCheck(int id){
    		buttonNumber = id;
    	}
    	public void update(){
    		buttonCheck = getRawButton(buttonNumber);
    		if(buttonCheck){
	    		if(buttonActive){
	    			if(System.currentTimeMillis() - buttonStartTime > 250){
	    				longPressActive = true;
	    			}
	    		}else{
	    			buttonActive = true;
	    			buttonStartTime = System.currentTimeMillis();
	    		}
    		}else{
    			if(buttonActive){
    				buttonActive = false;
    				if(longPressActive){
    					hasBeenPressed = false;
    					longPressActive = false;
    				}else{
    					hasBeenPressed = true;
    				}
    			}
    		}
    	}
    	public boolean wasPressed(){
    		if(hasBeenPressed){
    			hasBeenPressed = false;
    			return true;
    		}
    		return false;
    	}
    	public boolean longPressed(){
    		return longPressActive;
    	}
    	public boolean isBeingPressed(){
    		return buttonActive;
    	}
    }
    
    public void update(){
    	aButton.update();
    	bButton.update();
    	xButton.update();
    	yButton.update();
    	startButton.update();
    	backButton.update();
    	leftBumper.update();
    	rightBumper.update();
    }
}
