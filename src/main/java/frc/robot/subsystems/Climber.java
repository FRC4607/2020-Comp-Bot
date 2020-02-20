package frc.robot.subsystems;

import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.CURRENT_LIMIT;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.AlternateEncoderType;
//import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.ControlType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {

    // Hardware
    private final CANSparkMax mMaster;
    private final CANSparkMax mFollower;
    //private CANEncoder mAlternateEncoder;
    private CANPIDController mPIDController;
    private final DoubleSolenoid mShifter;


    // Hardware states
    private boolean mIsReversed;
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Climber.class );

    public void SetReversed ( boolean wantsReversed ) {
        if ( wantsReversed != mIsReversed ) {
            mIsReversed = wantsReversed;
            mMaster.setInverted( wantsReversed );
            mFollower.setInverted( wantsReversed );
            // current limiting for sparks by zero rpm, max rpm, and inbetween rpm current limits
            mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
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
            mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mLogger.info( "Gear set to: [High]" );
        } else if ( !wantsHighGear && mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kReverse );
            mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mLogger.info( "Gear set to: [Low]" );
        }
    }

    public boolean IsHighGear () {
        return mIsHighGear;
    }

    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        if (wantsBrakeMode && !mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            // current limiting for sparks by zero rpm, max rpm, and inbetween rpm current limits
            mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            // mLeftmaster.setNeutralMode( NeutralMode.Brake );
            // mLeftFollower.setNeutralMode( NeutralMode.Brake );
            // mRightMaster.setNeutralMode( NeutralMode.Brake );
            // mRightFollower.setNeutralMode( NeutralMode.Brake );
            mLogger.info( "Neutral mode set to: [Brake]" );

        } else if (!wantsBrakeMode && mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
            mFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
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

    public Climber ( CANSparkMax Master, CANSparkMax Follower,/* CANEncoder AlternateEncoder,*/ CANPIDController PidController,
                        DoubleSolenoid shifter ) {
  
        // Set the hardware
        mMaster = Master;
        mFollower = Follower;
        //mAlternateEncoder = AlternateEncoder;
        mPIDController = PidController;;        
        mShifter = shifter;

        // Current limiting
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
        mFollower.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );




        // Set the hardware states
        mIsHighGear = false;
        SetHighGear( true );
        mIsBrakeMode = false;
        SetBrakeMode( true );
        mIsReversed = false;
        SetReversed( true );
    }

    public static Climber create () {
        CANSparkMax Master =  SparkMax.CreateSparkMax( new CANSparkMax( CLIMBER.MASTER_ID, MotorType.kBrushless ) );
        CANSparkMax Follower =  SparkMax.CreateSparkMax( new CANSparkMax( CLIMBER.FOLLOWER_ID, MotorType.kBrushless ), Master );
        //CANEncoder AlternateEncoder = Master.getAlternateEncoder( AlternateEncoderType.kQuadrature, CLIMBER.SENSOR_COUNTS_PER_ROTATION );
        CANPIDController PidController = Master.getPIDController();
        DoubleSolenoid shifter = new DoubleSolenoid( GLOBAL.PCM_ID, CLIMBER.HIGH_GEAR_SOLENOID_ID, CLIMBER.LOW_GEAR_SOLENOID_ID );
        return new Climber( Master, Follower,/* AlternateEncoder,*/ PidController, shifter ); 
    }

    @Override
    public void periodic () {

    }

}
