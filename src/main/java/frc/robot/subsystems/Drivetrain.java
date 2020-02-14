package frc.robot.subsystems;

import frc.robot.Constants.MISC;
import frc.robot.lib.drivers.TalonSRX;
import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {

    // Hardware
    private final CANSparkMax mLeftMaster;
    private final CANSparkMax mLeftFollower;
    private final CANSparkMax mRightMaster;
    private final CANSparkMax mRightFollower;
    private CANEncoder mLeftAlternateEncoder;
    private CANPIDController mLeftPIDController;
    private CANEncoder mRightAlternateEncoder;
    private CANPIDController mRightPIDController;
    private final DoubleSolenoid mShifter;
    public final DifferentialDrive mDifferentialDrive;

    // Hardware states
    private boolean mIsReversed;
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Drivetrain.class );

    // reverse direction of driving
    public void SetReversed ( boolean wantsReversed ) {
        if ( wantsReversed != mIsReversed ) {
            mIsReversed = wantsReversed;
            mLeftMaster.setInverted( wantsReversed );
            mLeftFollower.setInverted( wantsReversed );
            mRightMaster.setInverted( wantsReversed) ;
            mRightFollower.setInverted( wantsReversed );
            mLogger.info( "Reversed drive set to: [{}]", mIsReversed );
        }
    }

    public boolean IsReversed () {
        return mIsReversed;
    }

    public void SetHighGear ( boolean wantsHighGear ) {
        if ( wantsHighGear && !mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kForward );
            mLogger.info( "Gear set to: [High]" );
        } else if ( !wantsHighGear && mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kReverse );
            mLogger.info( "Gear set to: [Low]" );
        }
    }

    public boolean IsHighGear () {
        return mIsHighGear;
    }

    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        if (wantsBrakeMode && !mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            // mLeftMaster.setNeutralMode( NeutralMode.Brake );
            // mLeftFollower.setNeutralMode( NeutralMode.Brake );
            // mRightMaster.setNeutralMode( NeutralMode.Brake );
            // mRightFollower.setNeutralMode( NeutralMode.Brake );
            mLogger.info( "Neutral mode set to: [Brake]" );

        } else if (!wantsBrakeMode && mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            // mLeftMaster.setNeutralMode( NeutralMode.Coast );
            // mLeftFollower.setNeutralMode( NeutralMode.Coast );
            // mRightMaster.setNeutralMode( NeutralMode.Coast );
            // mRightFollower.setNeutralMode( NeutralMode.Coast );
            mLogger.info( "Neutral mode set to: [Coast]" );
        }
    }

    public boolean IsBrakeMode () {
        return mIsBrakeMode;
    }

    public void OutputSmartDashboard () {
        if ( IsHighGear() ) {
          SmartDashboard.putString( "Gear", "High-Speed" );
        } else {
          SmartDashboard.putString( "Gear", "Low-Speed" );
        }

        if ( IsBrakeMode() ) {
            SmartDashboard.putString( "Neutral Mode", "Brake" );
        } else {
            SmartDashboard.putString( "Neutral Mode", "Coast" );
        }

        if( IsReversed() ) {
            SmartDashboard.putString( "Reversed Mode", "True" );
        } else {
            SmartDashboard.putString( "Reversed Mode", "False" );
        }
    }

    public Drivetrain ( CANSparkMax leftMaster, CANSparkMax leftFollower, CANEncoder leftAlternateEncoder, CANPIDController leftPidController,
                        CANSparkMax rightMaster, CANSparkMax rightFollower, CANEncoder rightAlternateEncoder, CANPIDController rightPidController,
                        DoubleSolenoid shifter ) {
  
        // Set the hardware
        mLeftMaster = leftMaster;
        mLeftFollower = leftFollower;
        mRightMaster = rightMaster; 
        mRightFollower = rightFollower;
        mLeftAlternateEncoder = leftAlternateEncoder;
        mLeftPIDController = leftPidController;
        mRightAlternateEncoder = rightAlternateEncoder;
        mRightPIDController = rightPidController;        
        mShifter = shifter;

        // Create differential drive object
        mDifferentialDrive = new DifferentialDrive( leftMaster, rightMaster );

        // Set the hardware states
        mIsHighGear = false;
        SetHighGear( true );
        mIsBrakeMode = false;
        SetBrakeMode( true );
        mIsReversed = false;
        SetReversed( true );
    }

    public static Drivetrain create () {
        // Talon's and Victor's go through a custom wrapper for creation
        WPI_TalonSRX leftMaster = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( DRIVETRAIN.DRIVETRAIN_LEFT_MASTER_ID) );
        WPI_TalonSRX leftFollower = TalonSRX.createTalonSRX( new WPI_TalonSRX( DRIVETRAIN.DRIVETRAIN_LEFT_FOLLOWER_ID), leftMaster );
        WPI_TalonSRX rightMaster = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( DRIVETRAIN.DRIVETRAIN_RIGHT_MASTER_ID) );
        WPI_TalonSRX rightFollower = TalonSRX.createTalonSRX( new WPI_TalonSRX( DRIVETRAIN.DRIVETRAIN_RIGHT_FOLLOWER_ID), rightMaster );
        DoubleSolenoid shifter = new DoubleSolenoid( MISC.PCM_ID, DRIVETRAIN.DRIVETRAIN_HIGH_GEAR_SOLENOID_ID, DRIVETRAIN.DRIVETRAIN_LOW_GEAR_SOLENOID_ID );
        CANEncoder leftAlternateEncoder = leftMaster.getAlternateEncoder( AlternateEncoderType.kQuadrature, DRIVETRAIN.SENSOR_COUNTS_PER_ROTATION );
        CANPIDController leftPidController = leftMaster.getPIDController();
        CANEncoder rightAlternateEncoder = rightMaster.getAlternateEncoder( AlternateEncoderType.kQuadrature, DRIVETRAIN.SENSOR_COUNTS_PER_ROTATION );
        CANPIDController rightPidController = rightMaster.getPIDController();
        return new Drivetrain( leftMaster, leftFollower, leftAlternateEncoder, leftPidController,
                               rightMaster, rightFollower, rightAlternateEncoder, rightPidController, shifter ); 
    }

    @Override
    public void periodic () {

    }

}
