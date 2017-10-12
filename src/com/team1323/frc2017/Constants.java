package com.team1323.frc2017;

public class Constants {
	public static final double kLooperDt = 0.005;
	
	public static final double kWheelDiameter = 4.0;//inches
	public static final double kTrackWidthInches = 26.655;
    public static final double kTrackScrubFactor = 0.924;
	
	public static final double kShootingSpeed = 2650.0;
	public static final double kShooterAllowableError = 100.0;//rpm
	
 // Path following constants
	public static final double kSegmentCompletionTolerance = 0.1; // inches
	public static final double kPathFollowingMaxAccel = 120.0; // inches per second^2
    public static final double kPathFollowingMaxVel = 120.0; // inches per second
    
 // Goal tracker constants
    public static final double kMaxGoalTrackAge = 1.0;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 30.0;
    public static final double kTrackReportComparatorStablityWeight = 1.0;
    public static final double kTrackReportComparatorAgeWeight = 1.0;
    
 // Pose of the camera frame w.r.t. the robot frame
    public static final double kCameraXOffset = 0.0;
    public static final double kCameraYOffset = 0.0;
    public static final double kCameraZOffset = 20.9;//TBM
    public static final double kCameraPitchAngleDegrees = 29.56; //TBM
    public static final double kCameraYawAngleDegrees = 0.0;
    public static final double kCameraDeadband = 0.0;
    
    public static final double kBoilerTargetTopHeight = 88.0;
    public static final double kBoilerRadius = 7.5;
    
 // Phone
    public static final int kAndroidAppTcpPort = 8254;
}
