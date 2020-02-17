package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.GLOBAL;
import frc.robot.lib.drivers.PressureSensor;
import frc.robot.lib.drivers.PDP;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TransferWheel;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.IntakeDrive;
import frc.robot.commands.HoodDrive;
import frc.robot.commands.FlywheelSpin;
import frc.robot.commands.TurretSpin;
import frc.robot.commands.Auto1;
import frc.robot.commands.Auto2;

import org.slf4j.Logger;

public class RobotContainer {

    // Hardware
    private final XboxController mDriverXbox = new XboxController( Constants.DRIVER_XBOX );
    private final XboxController mOperatorXbox = new XboxController( Constants.OPERATOR_XBOX );
    private final PressureSensor mPressureSensor = new PressureSensor( Constants.PRESSURE_SENSOR_ANALOG_CHANNEL, Constants.PRESSURE_SENSOR_VOLTS_AT_ZERO_PRESSURE, 
                                                                       Constants.PRESSURE_SENSOR_PRESSURE_PER_VOLT );
    private final PowerDistributionPanel mPDP = PDP.createPDP( new PowerDistributionPanel( GLOBAL.PDP_ID ), GLOBAL.PDP_ID );

     // Subsystems
     private Drivetrain mDrivetrain = Drivetrain.create();
     private Flywheel mFlywheel = Flywheel.create();
     private Hood mHood = Hood.create();
     private Hopper mHopper = Hopper.create();
     private Indexer mIndexer = Indexer.create();
     private Intake mIntake = Intake.create();
     private TransferWheel mTransferWheel = TransferWheel.create();
     private Shooter mSuperStructure = Shooter.create();
     private Turret mTurret = Turret.create(); 

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
 
     public void SetMatchState ( MatchState_t matchState ) {
         mMatchState = matchState;
     }
 
     public Command GetAutonomousCommand () {
        return mAutoChooser.getSelected();
    }

     // Button mappings
     private void ConfigureButtonBindings () {
        // drivetrian buttons
        new JoystickButton( mDriverXbox, 1).whenPressed( new InstantCommand( () -> mDrivetrain.SetHighGear( !mDrivetrain.IsHighGear() ), mDrivetrain ) );
        new JoystickButton( mDriverXbox, 4).whenPressed( new InstantCommand( () -> mDrivetrain.SetReversed( !mDrivetrain.IsReversed() ), mDrivetrain ) );
        // intake shifting
        new JoystickButton( mDriverXbox, 2).whenPressed( new InstantCommand( () -> mIntake.SetUp( !mIntake.IsUp() ), mDrivetrain ) ); 
        // Run hopper and indexer together on button a 
        new JoystickButton( mOperatorXbox, 1).whenPressed( new InstantCommand( () -> mHopper.Spin() ) ); 
        new JoystickButton( mOperatorXbox, 1).whenReleased( new InstantCommand( () -> mHopper.Stop() ) ); 
        new JoystickButton( mOperatorXbox, 1).whenPressed( new InstantCommand( () -> mIndexer.Spin() ) ); 
        new JoystickButton( mOperatorXbox, 1).whenReleased( new InstantCommand( () -> mIndexer.Stop() ) ); 
        // run hopper backward on x
        new JoystickButton( mOperatorXbox, 3).whenPressed( new InstantCommand( () -> mHopper.SpinBack() ) ); 
        new JoystickButton( mOperatorXbox, 3).whenReleased( new InstantCommand( () -> mHopper.Stop() ) ); 
        // run hopper backward on x
        new JoystickButton( mOperatorXbox, 3).whenPressed( new InstantCommand( () -> mIndexer.SpinBack() ) ); 
        new JoystickButton( mOperatorXbox, 3).whenReleased( new InstantCommand( () -> mIndexer.Stop() ) ); 
        // transfer wheel operator button b
        new JoystickButton( mOperatorXbox, 2).whenPressed( new InstantCommand( () -> mTransferWheel.Spin() ) ); 
        new JoystickButton( mOperatorXbox, 2).whenReleased( new InstantCommand( () -> mTransferWheel.Stop() ) ); 
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

    }

    // start vision output
    public void StartLimelight() {
        mTurret.mVision.mVisionThread.startPeriodic( 0.01 );
    }

    // stop vision output 
    public void StopLimelight() {
        mTurret.mVision.mVisionThread.stop();
    }

    public RobotContainer () {
        ConfigureButtonBindings();
        mDrivetrain.setDefaultCommand( new TeleopDrive( mDrivetrain, mDriverXbox ) );
        mIntake.setDefaultCommand( new IntakeDrive( mIntake, mDriverXbox ) );
        mFlywheel.setDefaultCommand( new FlywheelSpin( mFlywheel, mOperatorXbox ) );
        mHood.setDefaultCommand( new HoodDrive( mHood, mOperatorXbox ) );
        mTurret.setDefaultCommand( new TurretSpin( mTurret ) );
        mAutoChooser.setDefaultOption( "Auto 1", new Auto1( mDrivetrain, mFlywheel, mHopper, mIndexer, mTransferWheel ) );
        mAutoChooser.addOption( "Auto 2", new Auto2( mDrivetrain ) );
        SmartDashboard.putData( "Auto Chooser", mAutoChooser );
        mMatchState = MatchState_t.robotInit;
    }
 
}

