package com.team1323.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;
import com.team1323.frc2017.Ports;
import com.team1323.frc2017.loops.Loop;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearIntake extends Subsystem{
	private CANTalon intakeMotor;
	private Solenoid gearFlaps;
	private DigitalInput banner;
	
	private boolean hasGear = false;
	private boolean isScoring = false;
	public boolean isScoring(){
		return isScoring;
	}
	private boolean needsToNotifyDriver = false;
	public boolean needsToNotify(){
		if(needsToNotifyDriver){
			needsToNotifyDriver = false;
			return true;
		}
		return needsToNotifyDriver;
	}
	private boolean hasSeenGearWhileIntaking = false;
	private double firstTimeGearSeen = Double.POSITIVE_INFINITY;
	
	private static GearIntake instance = new GearIntake();
	public static GearIntake getInstance(){
		return instance;
	}
	
	public GearIntake(){
		intakeMotor = new CANTalon(Ports.GEAR_INTAKE);
		intakeMotor.setCurrentLimit(15);
		intakeMotor.EnableCurrentLimit(true);
		intakeMotor.setVoltageRampRate(48.0);
		intakeMotor.setStatusFrameRateMs(StatusFrameRate.Feedback, 1000);
		intakeMotor.changeControlMode(TalonControlMode.PercentVbus);
		gearFlaps = new Solenoid(20, Ports.GEAR_FLAPS);
		banner = new DigitalInput(0);
	}
	
	public enum State{
		INTAKING, HOLDING, SCORING, IDLE, REVERSED
	}
	private State currentState = State.HOLDING;
	private double stateStartTime = Double.POSITIVE_INFINITY;
	public State getState(){
		return currentState;
	}
	public void setState(State state){
		currentState = state;
		hasSeenGearWhileIntaking = false;
	}
	
	private final Loop loop = new Loop(){
		@Override
		public void onStart(double timestamp){
			setState(State.HOLDING);
		}
		
		@Override
		public void onLoop(double timestamp){
			synchronized (GearIntake.this){
				switch(currentState){
				case INTAKING:
					retractGearFlaps();
					intakeForward();
					if(!banner.get()){
						needsToNotifyDriver = true;
						hasGear = true;
						if(!hasSeenGearWhileIntaking)
							firstTimeGearSeen = timestamp;
						hasSeenGearWhileIntaking = true;
					}
					break;
				case HOLDING:
					retractGearFlaps();
					if(hasGear && timestamp - firstTimeGearSeen < 0.5){
						intakeForward();
					}else if(hasGear){
						intakeMotor.set(-0.07);
					}else{
						stopIntake();
					}
					break;
				case SCORING:
					hasGear = false;
					break;
				case IDLE:
					stopIntake();
					extendGearFlaps();
					break;
				case REVERSED:
					intakeReverse();
					break;
				default:
					break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp){
			setState(State.HOLDING);
		}
	};
	public Loop getLoop(){
		return loop;
	}
	
	public void intakeForward(){
		intakeMotor.set(-1.0);
	}
	
	public void intakeHold(){
		intakeMotor.set(0.25);
	}
	
	public void intakeReverse(){
		intakeMotor.set(1.0);
	}
	
	public void stopIntake(){
		intakeMotor.set(0);
	}
	
	public void extendGearFlaps(){
		gearFlaps.set(true);
	}
	
	public void retractGearFlaps(){
		gearFlaps.set(false);
	}
	
	@Override
	public synchronized void stop(){
		stopIntake();
		retractGearFlaps();
		setState(State.HOLDING);
	}
	
	public void score(){
		setState(State.SCORING);
		if(!isScoring){
			ScoringThread t = new ScoringThread();
			t.start();
		}
	}
	
	public class ScoringThread extends Thread{
		public void run(){
			isScoring = true;
			intakeForward();
			Timer.delay(0.1);
			extendGearFlaps();
			Timer.delay(0.5);
			stopIntake();
			setState(GearIntake.State.IDLE);
			isScoring = false;
		}
	}
	
	@Override
	public synchronized void zeroSensors(){
		//no-op
	}
	
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Gear Intake Voltage", intakeMotor.getOutputVoltage());
		//SmartDashboard.putNumber("Gear Intake Current", intakeMotor.getOutputCurrent());
		SmartDashboard.putBoolean("Gear Intake Banner", banner.get());
	}
}
