package frc.robot.subsystems;

import static frc.robot.Constants.DRIVETRAIN.CLOSED_LOOP_ERROR_RANGE;
import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants;
import frc.robot.Constants.CURRENT_LIMIT;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {

    public static enum ControlState_t {
        OpenLoop { @Override public String toString() { return "Open-Loop"; } },
        ClosedLoop { @Override public String toString() { return "Closed-Loop"; } },
    }

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
    private ControlState_t mControlState;
    private double mTargetVelocity_Units_Per_100ms;
    private double mTargetSpeed;

    // Hardware states
    private boolean mIsReversed;
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private boolean Turn;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Drivetrain.class );

    private final SimpleMotorFeedforward motorFeedForward = 
    new SimpleMotorFeedforward(DRIVETRAIN.kS, DRIVETRAIN.kV, DRIVETRAIN.kA);


    public void SetReversed ( boolean wantsReversed ) {
        if ( wantsReversed != mIsReversed ) {
            mIsReversed = wantsReversed;
            mLeftMaster.setInverted( wantsReversed );
            mLeftFollower.setInverted( wantsReversed );
            mRightMaster.setInverted( wantsReversed) ;
            mRightFollower.setInverted( wantsReversed );
            // current limiting for sparks by zero rpm, max rpm, and inbetween rpm current limits
            mLeftMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mLeftFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mRightMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mRightFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mLogger.info( "Reversed drive set to: [{}]", mIsReversed );
        }
    }


 /**
   * Prepare to shoot the given distance in INCHES
   * @param distanceToTarget distance in INCHES
   */
//   public void prepareToTurn( double distanceToTarget ) {
//     if ( distanceToTarget > 150 ) {
//         mTargetSpeed = 1.0;
//     } else if ( distanceToTarget <= 150 ) {
//         mTargetSpeed = .25;
//     }
    // mLeftPIDController.setReference(
    //     mTargetSpeed,
    //     ControlType.kVelocity,
    //     0,
    //     motorFeedForward.calculate( mTargetSpeed / 60, (mTargetSpeed - mLeftAlternateEncoder.getVelocity()) / 60 ) );
    // }

    public boolean isReadyToTurn() {
        return Math.abs( mLeftAlternateEncoder.getVelocity() - mTargetSpeed ) <= CLOSED_LOOP_ERROR_RANGE;
    }


    public void Turn() {
        mLeftMaster.set(1.0);
        Turn = true;
      }

    public void Stop() {
        mLeftMaster.set(0.0);
        mRightMaster.set(0.0);
        mLeftFollower.set(0.0);
        mRightFollower.set(0.0);
    }

     /**
    * @return ControlState_t The current control state of the flywheel.
    */
    public ControlState_t GetControlState () {
        return mControlState;
    }

    /**
     * 
     * @param desiredState
     */
    public void SetControlState( ControlState_t desiredState ) {
        mControlState = desiredState;
    }
    

    public boolean IsReversed () {
        return mIsReversed;
    }

    public void SetHighGear ( boolean wantsHighGear ) {
        if ( wantsHighGear && !mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kForward );
            mLeftMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mLeftFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mRightMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mRightFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mLogger.info( "Gear set to: [High]" );
        } else if ( !wantsHighGear && mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kReverse );
            mLeftMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mLeftFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mRightMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mRightFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mLogger.info( "Gear set to: [Low]" );
        }
    }

    public boolean IsHighGear () {
        return mIsHighGear;
    }

    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        mLeftMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
        mLeftFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
        mRightMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
        mRightFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
        if (wantsBrakeMode && !mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            // current limiting for sparks by zero rpm, max rpm, and inbetween rpm current limits
            mLeftMaster.setIdleMode( IdleMode.kBrake );
            mLeftFollower.setIdleMode( IdleMode.kBrake );
            mRightMaster.setIdleMode( IdleMode.kBrake );
            mRightFollower.setIdleMode( IdleMode.kBrake );
            mLogger.info( "Neutral mode set to: [Brake]" );

        } else if (!wantsBrakeMode && mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mLeftMaster.setIdleMode( IdleMode.kCoast );
            mLeftFollower.setIdleMode( IdleMode.kCoast );
            mRightMaster.setIdleMode( IdleMode.kCoast );
            mRightFollower.setIdleMode( IdleMode.kCoast );
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

        // Current limiting
        mLeftMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
        mLeftFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
        mRightMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
        mRightFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );

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
        CANSparkMax leftMaster =  SparkMax.CreateSparkMax( new CANSparkMax( DRIVETRAIN.LEFT_MASTER_ID, MotorType.kBrushless ) );
        CANSparkMax leftFollower =  SparkMax.CreateSparkMax( new CANSparkMax( DRIVETRAIN.LEFT_FOLLOWER_ID, MotorType.kBrushless ), leftMaster );
        CANEncoder leftAlternateEncoder = leftMaster.getAlternateEncoder( AlternateEncoderType.kQuadrature, DRIVETRAIN.SENSOR_COUNTS_PER_ROTATION );
        CANPIDController leftPidController = leftMaster.getPIDController();
        CANSparkMax rightMaster =  SparkMax.CreateSparkMax( new CANSparkMax( DRIVETRAIN.RIGHT_MASTER_ID, MotorType.kBrushless ) );
        CANSparkMax rightFollower =  SparkMax.CreateSparkMax( new CANSparkMax( DRIVETRAIN.RIGHT_FOLLOWER_ID, MotorType.kBrushless ), rightMaster );
        CANEncoder rightAlternateEncoder = rightMaster.getAlternateEncoder( AlternateEncoderType.kQuadrature, DRIVETRAIN.SENSOR_COUNTS_PER_ROTATION );
        CANPIDController rightPidController = rightMaster.getPIDController();
        DoubleSolenoid shifter = new DoubleSolenoid( GLOBAL.PCM_ID, DRIVETRAIN.HIGH_GEAR_SOLENOID_ID, DRIVETRAIN.LOW_GEAR_SOLENOID_ID );
        return new Drivetrain( leftMaster, leftFollower, leftAlternateEncoder, leftPidController,
                               rightMaster, rightFollower, rightAlternateEncoder, rightPidController, shifter ); 
    }

    @Override
    public void periodic () {

    }

}
