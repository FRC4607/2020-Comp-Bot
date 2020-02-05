package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.lib.drivers.PressureSensor;
import frc.robot.lib.drivers.Photoeye;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TransferWheel;
import frc.robot.subsystems.SuperStructure;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.Auto1;
import frc.robot.commands.Auto2;
import org.slf4j.Logger;

public class RobotContainer {

    // Hardware
    private final XboxController mDriverXbox = new XboxController( Constants.DRIVER_XBOX );
    private final XboxController mOperatorXbox = new XboxController( Constants.OPERATOR_XBOX );
    private final PressureSensor mPressureSensor = new PressureSensor( Constants.PRESSURE_SENSOR_ANALOG_CHANNEL, Constants.PRESSURE_SENSOR_VOLTS_AT_ZERO_PRESSURE, Constants.PRESSURE_SENSOR_PRESSURE_PER_VOLT );
    private final Photoeye mIndexerPhotoeye = new Photoeye( Constants.INDEXER_PHOTOEYE_ANALOG_CHANNEL );
    private final Photoeye mTransferPhotoeye = new Photoeye( Constants.TRANSFER_PHOTOEYE_ANALOG_CHANNEL );

     // Subsystems
     private final Drivetrain mDrivetrain = Drivetrain.create();
     private final Flywheel mFlywheel = Flywheel.create();
     private final Hood mHood = Hood.create();
     private final Hopper mHopper = Hopper.create();
     private final Indexer mIndexer = Indexer.create();
     private final Intake mIntake = Intake.create();
     private final TransferWheel mTransferWheel = TransferWheel.create();
     private final SuperStructure mSuperStructure = SuperStructure.create();
     
     // Autonomous chooser
     //private final TeleopDrive mTeleopDrive = new TeleopDrive( mDrivetrain, mDriverXbox, mDriverJoystickThrottle, mDriverJoystickTurn );
     private final SendableChooser<Command> mAutoChooser = new SendableChooser<>();
 
     // Match states for debug data output
     public static enum MatchState_t {
         robotInit {
             @Override
             public String toString() {
                 return "Robot Init";
             }
         },
         disabledInit {
             @Override
             public String toString() {
                 return "Disabled Init";
             }
         },
         autonomousInit {
             @Override
             public String toString() {
                 return "Autonomous Init";
             }
         },
         autonomousPeriodic {
             @Override
             public String toString() {
                 return "Autonomous Periodic";
             }
         },
         teleopInit {
             @Override
             public String toString() {
                 return "Teleop Init";
             }
         },
         teleopPeriodic {
             @Override
             public String toString() {
                 return "Teleop Periodic";
             }
         };
     }
     
     private MatchState_t mMatchState;
 
     public MatchState_t GetMatchState () {
         return mMatchState;
     }
 
     public void SetMatchState (MatchState_t matchState) {
         mMatchState = matchState;
     }
 
     // Debug logging
     public void LogRobotDataHeader (Logger robotLogger) {
         robotLogger.debug( "Time,Match State,Pressure (PSI)" );
     }   
     public void LogRobotDataToRoboRio (Logger robotLogger) {
         robotLogger.debug( "{},{},{}", Timer.getFPGATimestamp(), mMatchState.toString(), mPressureSensor.GetPressureInPSI() );
     }
 
     // Smartdashboard output
     public void UpdateSmartDashboard() {
         SmartDashboard.putNumber( "Pressure Sensor (PSI)", mPressureSensor.GetPressureInPSI() );
         mDrivetrain.OutputSmartDashboard();
     }
 
     // Button mappings
     private void ConfigureButtonBindings () {
         new JoystickButton( mDriverXbox, 1).whenPressed( new InstantCommand( () -> mDrivetrain.SetHighGear( !mDrivetrain.IsHighGear() ), mDrivetrain ) );
         new JoystickButton( mDriverXbox, 4).whenPressed( new InstantCommand( () -> mDrivetrain.SetReversed( !mDrivetrain.IsReversed() ), mDrivetrain ) );
        //  mDriverJoystickThrottleButton.whenPressed( new InstantCommand( () -> mDrivetrain.SetHighGear( !mDrivetrain.IsHighGear() ), mDrivetrain ) );
        //  mDriverJoystickTurnButton.whenPressed( new InstantCommand( () -> mDrivetrain.SetReversed( !mDrivetrain.IsReversed() ), mDrivetrain ) );
     }
  
     public Command GetAutonomousCommand () {
         return mAutoChooser.getSelected();
     }
 
     public RobotContainer () {
         ConfigureButtonBindings();
         mDrivetrain.setDefaultCommand( new TeleopDrive( mDrivetrain, mDriverXbox ) );
         mAutoChooser.setDefaultOption( "Auto 1", new Auto1( mDrivetrain ) );
         mAutoChooser.addOption( "Auto 2", new Auto2( mDrivetrain ) );
         SmartDashboard.putData( "Auto Chooser", mAutoChooser );
         mMatchState = MatchState_t.robotInit;
     }
 
 }
 