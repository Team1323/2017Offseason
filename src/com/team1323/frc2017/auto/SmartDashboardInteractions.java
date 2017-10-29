package com.team1323.frc2017.auto;

import com.team1323.frc2017.auto.modes.BlueHopperMode;
import com.team1323.frc2017.auto.modes.BlueMiddleGearAndShootMode;
import com.team1323.frc2017.auto.modes.RedHopperMode;
import com.team1323.frc2017.auto.modes.RedMiddleGearAndShootMode;
import com.team1323.frc2017.auto.modes.StandStillMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
	private static final String AUTO_OPTIONS = "auto_options";
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    private static final String AUTO_SIDE = "auto_side";
    private static final String SELECTED_AUTO_SIDE = "selected_auto_side";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.HOPPER;
    private static final AutoSide DEFAULT_SIDE = AutoSide.BLUE;
    
    private SendableChooser modeChooser;
    private SendableChooser sideChooser;
    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser();
    	modeChooser.addDefault("Hopper", DEFAULT_MODE);
    	modeChooser.addObject("Middle Gear", AutoOption.MIDDLE_GEAR);
    	sideChooser = new SendableChooser();
    	sideChooser.addDefault("Blue", DEFAULT_SIDE);
    	sideChooser.addObject("Red", AutoSide.RED);
    	SmartDashboard.putData("Mode Chooser", modeChooser);
    	SmartDashboard.putData("Side Chooser", sideChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
    	SmartDashboard.putString(SELECTED_AUTO_SIDE, DEFAULT_SIDE.color);
    }
    
    public AutoModeBase getSelectedAutoMode(){
        AutoOption selectedOption =  (AutoOption)  modeChooser.getSelected();
        
        AutoSide selectedSide = (AutoSide) sideChooser.getSelected();
        
        return createAutoMode(selectedOption, selectedSide);
    }
    
    public String getSelectedSide(){
    	AutoSide side = (AutoSide) sideChooser.getSelected();
    	return side.color;
    }
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }
    
    enum AutoOption{
    	HOPPER("Hopper"),
    	MIDDLE_GEAR("Middle Gear"),
    	STAND_STILL("Stand Still");
    	
    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }
    
    public enum AutoSide{
    	BLUE("Blue"),RED("Red");
    	
    	public final String color;
    	
    	AutoSide(String color){
    		this.color = color;
    	}
    }
    
    private AutoModeBase createAutoMode(AutoOption option, AutoSide side){
    	switch(option){
    		case HOPPER:
    			if(side == AutoSide.BLUE){
    				return new BlueHopperMode();
    			}else{
    				return new RedHopperMode();
    			}
    		case MIDDLE_GEAR:
    			if(side == AutoSide.BLUE){
    				return new BlueMiddleGearAndShootMode();
    			}else{
    				return new RedMiddleGearAndShootMode();
    			}
    		case STAND_STILL: // fallthrough
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    	SmartDashboard.putString(SELECTED_AUTO_SIDE, getSelectedSide());
    }
}
