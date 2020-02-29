package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Constants.PRESSURE_SENSOR;
import frc.robot.Constants.GLOBAL;
import frc.robot.lib.drivers.PressureSensor;
import frc.robot.lib.drivers.Limelight;
import frc.robot.lib.drivers.PDP;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TransferWheel;
import frc.robot.subsystems.Turret;
import frc.robot.util.XboxControllerAxisButton;
import frc.robot.subsystems.Climber;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.IntakeDrive;
import frc.robot.commands.LimelightDrive2;
import frc.robot.commands.HoodDrive;
import frc.robot.commands.TransferWheelRun;
import frc.robot.commands.FlywheelToSetRPM;
import frc.robot.commands.TurretLimelight;
import frc.robot.commands.TurretManual;
import frc.robot.commands.Auto1;
import frc.robot.commands.Auto2;
import frc.robot.commands.Auto3_0;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.DrivetrainZeroEncoders;
import frc.robot.lib.controllers.Vision;
import org.slf4j.Logger;

public class RobotContainer {

    // Hardware
    private final XboxController mDriverXbox = new XboxController( CONTROLLER.DRIVER_XBOX );
    private final XboxController mOperatorXbox = new XboxController( CONTROLLER.OPERATOR_XBOX );
    private final PressureSensor mPressureSensor = new PressureSensor( PRESSURE_SENSOR.PRESSURE_SENSOR_ANALOG_CHANNEL, PRESSURE_SENSOR.PRESSURE_SENSOR_VOLTS_AT_ZERO_PRESSURE, 
                                                                       PRESSURE_SENSOR.PRESSURE_SENSOR_PRESSURE_PER_VOLT );
    private final PowerDistributionPanel mPDP = PDP.createPDP( new PowerDistributionPanel( GLOBAL.PDP_ID ), GLOBAL.PDP_ID );
    private Limelight mLimelight = new Limelight();
    // private UsbCamera mCamera = new UsbCamera( "USB Camera", 1 ); 
    // private CameraServer mCameraServer = new CameraServer.getInstance().startAutomaticCapture();


    // Subsystems 
    public static Drivetrain mDrivetrain = Drivetrain.create();
    private Flywheel mFlywheel = Flywheel.create();
    private Hood mHood = Hood.create();
    private Hopper mHopper = Hopper.create();
    private Indexer mIndexer = Indexer.create();
    private Intake mIntake = Intake.create();
    private TransferWheel mTransferWheel = TransferWheel.create();
    // private Shooter mShooter = Shooter.create();
    private Turret mTurret = Turret.create(); 
    private Climber mClimber = Climber.create(); 
    private Vision mVision = Vision.create();
    private DoubleSolenoid mShifter;
  
    // Autonomous chooser 
    private final SendableChooser<Command> mAutoChooser = new SendableChooser<>();
 
    // Match states for debug data output
    public static enum MatchState_t {
        robotInit { @Override public String toString() { return "Robot Init"; } },
        robotPeriodic { @Override public String toString() { return "Robot Periodic"; } },
        disabledInit { @Override public String toString() { return "Disabled Init"; } },
        autonomousInit { @Override public String toString() { return "Autonomous Init"; } },
        autonomousPeriodic { @Override public String toString() { return "Autonomous Periodic"; } },
        teleopInit { @Override public String toString() { return "Teleop Init"; } },
        teleopPeriodic { @Override public String toString() { return "Teleop Periodic"; } };
    }

    private MatchState_t mMatchState;
 
    public MatchState_t GetMatchState () {
        return mMatchState;
    }
 
    // private Limelight Limelight() {
    //     return null;
    // }

    public void SetMatchState(MatchState_t matchState) {
        mMatchState = matchState;
    }

    public Command GetAutonomousCommand () {
        return mAutoChooser.getSelected();
    }

    // Button mappings
    private void ConfigureButtonBindings () {
        SetUpmDriverXbox();
        SetUpmOperatorXbox();
        SetUpSmartDashboardButtons();
        SetUpSubsystemData();
    } 

    private void SetUpSubsystemData() {
        SmartDashboard.putData( "DT", mDrivetrain );
        SmartDashboard.putData( mFlywheel );
        SmartDashboard.putData( mHood );
        SmartDashboard.putData( mIndexer );
        SmartDashboard.putData( mIntake );
        SmartDashboard.putData( mTransferWheel );
        SmartDashboard.putData( mClimber );
        SmartDashboard.putData( mHopper );
        SmartDashboard.putData( mTurret ); 

    }

    private void SetUpSmartDashboardButtons() {
        // SmartDashboard.putData( new Drive_Turn_To_Setpoint( mDrivetrain,mDriverXbox,90.0 ) );
        SmartDashboard.putData( new DrivetrainZeroEncoders( mDrivetrain ) );
        SmartDashboard.putData( new DriveForDistance(mDrivetrain, 5, -0.75, 0.0) );
        // SmartDashboard.putData( new Auto3_0(mDrivetrain, mFlywheel, mHopper, mIndexer, mTransferWheel, mLimelight) );
        // SmartDashboard.putData( new Auto3_1(mDrivetrain, mFlywheel, mHopper, mIndexer, mTransferWheel, mIntake /*, mLimelight*/) );

    }

    private void SetUpmOperatorXbox() {

        // HOPPER AND INDEXER CONTROL
        // Run hopper and indexer together on button a 
        new JoystickButton( mOperatorXbox, 1 ).whenPressed( new InstantCommand( () -> mHopper.Spin() ) ); 
        new JoystickButton( mOperatorXbox, 1 ).whenReleased( new InstantCommand( () -> mHopper.Stop() ) ); 
        new JoystickButton( mOperatorXbox, 1 ).whenPressed( new InstantCommand( () -> mIndexer.Spin() ) ); 
        new JoystickButton( mOperatorXbox, 1 ).whenReleased( new InstantCommand( () -> mIndexer.Stop() ) ); 
        // run hopper and indexer backward on x
        new JoystickButton( mOperatorXbox, 3 ).whenPressed( new InstantCommand( () -> mHopper.SpinBack() ) ); 
        new JoystickButton( mOperatorXbox, 3 ).whenReleased( new InstantCommand( () -> mHopper.Stop() ) ); 
        new JoystickButton( mOperatorXbox, 3 ).whenPressed( new InstantCommand( () -> mIndexer.SpinBack() ) ); 
        new JoystickButton( mOperatorXbox, 3 ).whenReleased( new InstantCommand( () -> mIndexer.Stop() ) ); 
        
        // TRANSFER WHEEL CONTROL 
        // transfer wheel operator button b
        // new JoystickButton( mOperatorXbox, 2 ).whenHeld( new TransferWheelRun( mTransferWheel, mFlywheel, mLimelight, true ) );
        new JoystickButton( mOperatorXbox, 4 ).whenHeld( new TransferWheelRun( mTransferWheel, mFlywheel, mLimelight, false ) );
        new JoystickButton( mOperatorXbox, 2 ).whenPressed( new InstantCommand( () -> mTransferWheel.Spin() ) ); 
        new JoystickButton( mOperatorXbox, 2 ).whenReleased( new InstantCommand( () -> mTransferWheel.Stop() ) );

        // FLYWHEEL CONTROL
        // flywheel closed loop
        // right trigger
        new XboxControllerAxisButton( mOperatorXbox, 3 ).whileHeld( new FlywheelToSetRPM( mFlywheel, mOperatorXbox ) );

        // turret 
        // turret limelight control on left bumper 
        new JoystickButton( mOperatorXbox, 5 ).whenPressed( new InstantCommand( () -> mVision.setLimelightLEDOn() ) ); 
        new JoystickButton( mOperatorXbox, 5 ).whileHeld( new TurretLimelight( mTurret ) ); 
        new JoystickButton( mOperatorXbox, 5 ).whenReleased( new InstantCommand( () -> mVision.setLimelightLEDOff() ) ); 
        //new XboxControllerAxisButton( mOperatorXbox, 1,1).whileHeld( new TurretManual( mTurret, mOperatorXbox ) );
        // left joystick, press at the same time
        // new JoystickButton( mOperatorXbox, 9 ).whileHeld( new TurretManual( mTurret, mOperatorXbox ) );

    }

    private void SetUpmDriverXbox() {

        // drivetrain buttons
        // button a
        new JoystickButton( mDriverXbox, 1 ).whenPressed( new InstantCommand( () -> mDrivetrain.SetHighGear( !mDrivetrain.IsHighGear() ), mDrivetrain ) );
        // button y
        new JoystickButton( mDriverXbox, 4 ).whenPressed( new InstantCommand( () -> mDrivetrain.SetReversed( !mDrivetrain.IsReversed() ), mDrivetrain ) );
        // intake shifting
        // button b
        new JoystickButton( mDriverXbox, 2 ).whenPressed( new InstantCommand( () -> mIntake.SetUp( !mIntake.IsUp() ), mDrivetrain ) ); 
        // limelight drive
        // button x
        new JoystickButton( mDriverXbox, 3 ).whileHeld( new LimelightDrive2( mDrivetrain, mDriverXbox, mLimelight ) ); 

        // CLIMBER CONTROLS
        // shift the climber up and down with pneumatics on start and select 
        // shift up on start 
        new JoystickButton( mDriverXbox, 8 ).whenPressed( new InstantCommand( () -> mClimber.SetLocked( !mClimber.IsLocked() ), mClimber ) ); 
        // shift down and drive motors on select
        new JoystickButton( mDriverXbox, 7 ).whenPressed( new InstantCommand( () -> mClimber.SetLocked( !mClimber.IsLocked() ), mClimber ) ); 
        new JoystickButton( mDriverXbox, 7 ).whileHeld( new InstantCommand( () -> mClimber.OpenLoop() ) ); 
        new JoystickButton( mDriverXbox, 7 ).whenReleased( new InstantCommand( () -> mClimber.Stop() ) ); 

    }

    // Debug logging 
    public void LogRobotDataHeader ( Logger fileLogger ) {
        fileLogger.debug( "Time,"+
                          "Match State,"+
                          "Flywheel State,"+
                          "Flywheel Ouput State,"+
                          "Flywheel Error State,"+
                          "Flywheel Target Percent Output,"+
                          "Flywheel Target Velocity (RPM),"+
                          "Flywheel Measured Velocity (RPM),"+
                          "Flywheel Velocity Error (RPM),"+
                          "Turret State,"+
                          "Turret Ouput State,"+
                          "Turret Error State,"+
                          "Turret Target Percent Output,"+
                          "Turret Target Position,"+
                          "PDP Voltage,"
                          //"PDP Slot 0 Current"                          
                          );
    }   
    
    public void LogRobotDataToRoboRio ( Logger fileLogger ) {
        fileLogger.debug( "{},{},{},{},{},{},{},{},{},{},{}",
                          Timer.getFPGATimestamp(),
                          mMatchState.toString(),
                          mFlywheel.GetFlywheelState().toString(),
                          mFlywheel.GetControlState().toString(),
                          mFlywheel.GetFailingState().toString(),
                          mFlywheel.GetTargetPercentOutput(),
                          mFlywheel.GetTargetVelocity_RPM(),
                          mFlywheel.GetCurrentVelocity_RPM(),
                          mFlywheel.GetError_RPM(),
                          mTurret.GetTurretState().toString(),
                          mTurret.GetControlState().toString(),
                          mTurret.GetFailingState().toString(),
                          mTurret.GetTargetPosition_Rot(),
                          mTurret.GetCurrentPosition_Rot(),
                          mTurret.GetTargetPercentOutput(),
                          mPDP.getVoltage()
                          //mPDP.getCurrent(  )                          
                          );
    }

    // Smartdashboard output
    public void UpdateSmartDashboard () {
        SmartDashboard.putNumber( "Pressure Sensor (PSI)", mPressureSensor.GetPressureInPSI() );
        mDrivetrain.OutputSmartDashboard();
        mTurret.mVision.mVisionThread.startPeriodic( 0.01 );
        mLimelight.getStream();
        mFlywheel.GetCurrentVelocity_RPM();
        mDrivetrain.IsBrakeMode();
        mIntake.IsUp(); 

    }

    // // start vision output
    // public void StartLimelight() {
    //    mTurret.mVision.mVisionThread.startPeriodic( 0.01 );
    // }

    // // stop vision output 
    // public void StopLimelight() {
    //     mTurret.mVision.mVisionThread.stop();
    // } 

    public RobotContainer () {
        ConfigureButtonBindings();
        mDrivetrain.setDefaultCommand( new TeleopDrive( mDrivetrain, mDriverXbox ) );
        mIntake.setDefaultCommand( new IntakeDrive( mIntake, mDriverXbox ) );
        mTurret.setDefaultCommand( new TurretManual( mTurret, mOperatorXbox ) );
        // mFlywheel.setDefaultCommand( new FlywheelToSetRPM( mFlywheel, mOperatorXbox ) );
        mHood.setDefaultCommand( new HoodDrive( mHood, mOperatorXbox ) );
        // mTurret.setDefaultCommand( new TurretSpin( mTurret ) );
        mAutoChooser.setDefaultOption( "Auto 3_0", new Auto3_0( mDrivetrain, mFlywheel, mHopper, mIndexer, mTransferWheel, mShifter ) );
        // mAutoChooser.setDefaultOption( "Auto 1", new Auto1( mDrivetrain, mFlywheel, mHopper, mIndexer, mTransferWheel ) );
        mAutoChooser.addOption( "Auto 2", new Auto2( mDrivetrain ) );
        // mAutoChooser.addOption( "Auto 3_0", new Auto3_0( mDrivetrain, mFlywheel, mHopper, mIndexer, mTransferWheel, mLimelight ) );
        SmartDashboard.putData( "Auto Chooser", mAutoChooser );
        mMatchState = MatchState_t.robotInit;
    }
 
}

