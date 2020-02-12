package frc.robot.subsystems;

import frc.robot.Constants.INDEXER;
import frc.robot.Constants.GLOBAL;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.drivers.Photoeye;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Indexer extends SubsystemBase {

    public static enum IndexerState_t {
        Init { @Override public String toString() { return "Init"; } },
        Loading { @Override public String toString() { return "Loading"; } },
        Holding { @Override public String toString() { return "Holding"; } },
        Fail { @Override public String toString() { return "Fail"; } };
    }

    public static enum ControlState_t {
        ClosedLoop { @Override public String toString() { return "Closed-Loop"; } };
    }

    public static enum FailingState_t {
        Ok { @Override public String toString() { return "Ok"; } }, 
        BrokenSensor { @Override public String toString() { return "Broken sensor"; } },      
        PowerLoss { @Override public String toString() { return "Power loss"; } },
        SeekTimeout { @Override public String toString() { return "Seek timeout"; } },
        SeekRetries { @Override public String toString() { return "Seek retries"; } };
    }

    // Hardware
    private final CANSparkMax mIndexerMotor;
    private final Photoeye mIndexerPhotoeye = new Photoeye( INDEXER.PHOTOEYE_ANALOG_CHANNEL );
    private StickyFaults mIndexerMotorFaults;

    // Closed-loop control
    private double mP, mI, mD, mFF;
    private CANPIDController mPIDController;
    private CANEncoder mHallSensor;
    private double mTargetVelocity_Units_Per_100ms;
    private double mTargetVelocity_RPM;
    private double mCurrentVelocity_RPM;
    private double mError_RPM;
   
    // State variables
    private IndexerState_t mIndexerState;
    private ControlState_t mControlState;
    private FailingState_t mFailingState;
    private double mSeekTimer_S;
    private int mSeekRetries;

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    public CANEncoder getEncoder() {
		return getEncoder(EncoderType.kHallSensor, 0);
	}

    /**
    * @return IndexerState_t The current state of the indexer.
    */
    public IndexerState_t GetFlywheelState () {
        return mIndexerState;
    }

    /**
    * @return ControlState_t The current control state of the flywheel.
    */
    public ControlState_t GetControlState () {
        return mControlState;
    }

    /**
    * @return FailingState_t The current failing state of the flywheel.
    */
    public FailingState_t GetFailingState () {
        return mFailingState;
    }

    /**
    * @return double The current closed-loop velocity target.
    */
    public double GetTargetVelocity_RPM () {
        return mTargetVelocity_RPM;
    }

    /**
    * @return double The current current velocity of the flywheel in RPM.
    */
    public double GetCurrentVelocity_RPM () {
        return mCurrentVelocity_RPM;
    }

    /**
    * @return ControlState_t The current error of the closed-loop velocity control.
    */
    public double GetError_RPM () {
        return mError_RPM;
    }

    /**
    * @param double The target velocity in RPM for closed-loop velocity control.
    */
    public void SetTargetVelocity_RPM ( double targetVelocity_RPM ) {
        mSeekRetries = 0;
        mTargetVelocity_RPM = targetVelocity_RPM;
        mTargetVelocity_Units_Per_100ms = targetVelocity_RPM * INDEXER.SENSOR_UNITS_PER_ROTATION / 600.0;
    }


    //-----------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------

 /**
    * This method will initialize the Indexer subsystem by setting the up the
    * master TalonSRX for velocity control and setting all of the internal
    * states to their starting values.
    */
    private void Initialize () {
        mPIDController.setFeedbackDevice( mHallSensor );
        mHallSensor.setInverted( false );
        mIndexerMotor.setNeutralMode( NeutralMode.Brake );
        mIndexerState = IndexerState_t.Init;
        mControlState = ControlState_t.ClosedLoop;
        mFailingState = FailingState_t.Ok;
        mCurrentVelocity_RPM = Double.NaN;
        mError_RPM = 0;
        mP = INDEXER.PID_KP;
        mI = INDEXER.PID_KI;
        mD = INDEXER.PID_KD;
        mFF = INDEXER.PID_KFF;     
        SetTargetVelocity_RPM( 0.0 );  // 0's mSeekRetries
        SetGains();
        SetVelocityOutput();
    }


    /**
    * This method will set the P-gain of the velocity controller.
    */  
    private void SetP ( double p ) {
        mP = p;
    }
    
    /**
    * This method will get the P-gain of the velocity controller.
    */  
    private double GetP () {
        return mP;
    }

    /**
    * This method will set the I-gain of the velocity controller.
    */  
    private void SetI ( double i ) {
        mI = i;
    }

    /**
    * This method will get the I-gain of the velocity controller.
    */  
    private double GetI () {
        return mI;
    }


    /**
    * This method will set the D-gain of the velocity controller.
    */  
    private void SetD ( double d ) {
        mD = d;
    }

    /**
    * This method will get the D-gain of the velocity controller.
    */  
    private double GetD () {
        return mD;
    }

    /**
    * This method will set the feedforward gain of the velocity controller.
    */  
    private void SetF ( double ff ) {
        mFF = ff;
    }

    /**
    * This method will get the feedforward gain of the velocity controller.
    */  
    private double GetFF () {
        return mFF;
    }

    /**
    * This method will set the velocity closed-loop gains of the TalonSRX.
    */
    private void SetGains () {
        mPIDController.setP( mP, INDEXER.PID_IDX );
        mPIDController.setI( mI, INDEXER.PID_IDX );
        mPIDController.setD( mD, INDEXER.PID_IDX );
        mPIDController.setFF( mFF, INDEXER.PID_IDX );
        mPIDController.setIZone( 0.0, INDEXER.PID_IDX );
        mPIDController.setOutputRange( -1.0, 1.0 );
        //mPIDController.setSmartMotionMaxVelocity( mMaxVelocity, INDEXER.PID_IDX );
        mPIDController.setSmartMotionMinOutputVelocity( 0.0, INDEXER.PID_IDX );
        //mPIDController.setSmartMotionMaxAccel( mMaxAcceleration, INDEXER.PID_IDX );
        mPIDController.setSmartMotionAllowedClosedLoopError( 0.1, INDEXER.PID_IDX );
    }

    /**
    * <b>This function should only be used for Livewindow PID tuning!<b> 
    * @return boolean True if the current control state is closed-loop
    */
    private boolean IsClosedLoop () {
        return mControlState == ControlState_t.ClosedLoop;
    }


/**
    * <b>This function should only be used for Livewindow PID tuning!<b> 
    */    
    private void EnableClosedLoop ( boolean wantsClosedLoop ) {
        if ( wantsClosedLoop && !IsClosedLoop() ) {
            SetGains();
            mControlState = ControlState_t.ClosedLoop;
        } else if ( !wantsClosedLoop && IsClosedLoop() ) {
            SetTargetPercentOutput( 0.0 );
            mControlState = ControlState_t.OpenLoop;
        }
    }

    /**
    * This method will set the mSeekTimer_S variable to the current time in seconds.
    */
    private void SetSeekTimer () {
        mSeekTimer_S = Timer.getFPGATimestamp();
    }

    /**
    * This method will check the sticky faults of the master TalonSRX for faults related to the sensor used for
    * velocity feedback.
    * @return boolean Logical OR of SensorOverflow, SensorOutOfPhase, and RemoteLossOfSignal
    */
    private boolean SensorIsBroken () {
        mIndexerMotor.getStickyFaults( mIndexerMotorFaults );
        return mIndexerMotorFaults.SensorOverflow | mIndexerMotorFaults.SensorOutOfPhase | mIndexerMotorFaults.RemoteLossOfSignal;
    }

    /**
    * This method will check the sticky faults of the master and follower TalonSRX for faults related to the power
    * supplied. 
    * @return boolean Logical OR of ResetDuringEn and UnderVoltage for both TalonSRX's
    */    
    private boolean PowerDisruption () {
        mIndexerMotor.getStickyFaults( mIndexerMotorFaults );
        return mIndexerMotorFaults.ResetDuringEn | mIndexerMotorFaults.UnderVoltage;
    }

    /**
    * This method will set output mode of the TalonSRX's to velocity at a target defined by the variable
    * mTargetVelocity_Units_Per_100ms.
    */    
    private void SetVelocityOutput () {
        mIndexerMotor.set( ControlState.Velocity, mTargetVelocity_Units_Per_100ms );
    }

    /**
    * This method will set the TalonSRX output based on the control state.
    */
    private void MotorOutput () {  
        switch ( mControlState ) {
            case ClosedLoop:
                SetVelocityOutput();
                break;
        }
    }


    /**
    * This method will update the states of the variable mFlywheelState, mControlState, and mFailingState.  After
    * the states have been updated, the TalonSRX outputs are set for either the open-loop (percentage output) or
    * closed-loop (velocity control) cases.
    * <p> 
    * This method should be set as the subsystems default command.  Furthermore, this subsystem isn't designed to be
    * command requirement, so commands should set this subsystem as a requirement.  Following these two rules will
    * ensure that this method is called after all of the subsystem's period() methods and all of the command's have
    * finished running.
    * <p>
    * The flow for subsystems with this design pattern is:
    * <ol>
    * <li>Put all sensor readings into the subsystem's periodic() method so they are gathered right away
    * <li>Commands generate new targets for the subsystem and set them up in the subsystem via set() method call
    * <li>All default subsystem commands will be run to update internal states and set their outputs
    * <li>Logging will grab all relevant data for logging and outputing to the SmartDashboard
    * </ol>
    * @see {@link edu.wpi.first.wpilibj2.command.CommandScheduler#run}
    */     
    public void Update () {
        switch ( mIndexerState ) {
            case Init:
                // No velocity command has come in, so just hold at 0.0
                if ( mTargetVelocity_Units_Per_100ms == 0.0 ) {
                    mIndexerState = IndexerState_t.Holding;
                
                // Set the seek timer and begin seeking to the velocity target
                } else  {
                    mIndexerState = IndexerState_t.Loading;
                }
                break;

            case Loading:
                // The velocity is within tolerance to move to the holding state...which is a go for shooting
                if ( Math.abs( mError_RPM ) <= INDEXER.OFFTRACK_ERROR_PERCENT * mTargetVelocity_RPM ) {
                    mIndexerState = IndexerState_t.Holding;
                }
                    // The retries have been exhausted
                    if ( mSeekRetries > INDEXER.SEEK_RETRY_LIMIT ) {
                        mControlState = ControlState_t.OpenLoop;
                        mFailingState = FailingState_t.SeekRetries;
                    } else {
                        mFailingState = FailingState_t.SeekTimeout;
                    }
                }
                break;

            case Holding:
                // The velocity has fallen outside of tolerance, move back to the seeking state
                if ( Math.abs( mError_RPM ) > INDEXER.OFFTRACK_ERROR_PERCENT * mTargetVelocity_RPM ) {
                    mIndexerState = INDEXERState_t.Holding;
                    SetSeekTimer();
                }
                break;

            case Fail:
                switch ( mFailingState ) {
                    // Failure due to broken sensor, keep checking if it comes back online
                    case BrokenSensor:
                    mIndexerMotor.clearStickyFaults( 0 ); // Send command whithout checking or blocking
                        if ( !SensorIsBroken() ) {
                            Initialize();
                        }
                        break;

                    case PowerLoss:
                    mIndexerMotor.clearStickyFaults( 0 ); // Send command whithout checking or blocking
                        if ( !PowerDisruption() ) {
                            Initialize();
                        }
                        break;

                    // Failure due to timing out during a seek
                    case SeekTimeout:
                        mFailingState = FailingState_t.Ok;
                        mIndexerState = IndexerState_t.Seeking;
                        SetSeekTimer();
                        break;
                    
                    // All of the seek retries have been exhausted
                    case SeekRetries:
                        break;

                    default:
                        break;

                }
                break;
        
        }


    // Hardware states
    private boolean mIsBrakeMode;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Indexer.class );


    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        if (wantsBrakeMode && !mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mIndexerMotor.setIdleMode(IdleMode.kBrake);
      mLogger.info("Neutral mode set to: [Brake]");

    } else if (!wantsBrakeMode && mIsBrakeMode) {
      mIsBrakeMode = wantsBrakeMode;
      mIndexerMotor.setIdleMode(IdleMode.kCoast);
            mLogger.info( "Neutral mode set to: [Coast]" );
        }
    }

    public boolean IsBrakeMode () {
        return mIsBrakeMode;
    }

    public void OutputSmartDashboard () {
        if ( IsBrakeMode() ) {
            SmartDashboard.putString( "Neutral Mode", "Brake" );
        } else {
            SmartDashboard.putString( "Neutral Mode", "Coast" );
        }
    }

    public Indexer ( CANSparkMax indexerMotor ) {
  
        // Set the hardware
        mIndexerMotor = indexerMotor;

        // Set the hardware states
        mIsBrakeMode = false;
        SetBrakeMode( true );
    }

    public static Indexer create () {
        // Talon's and Victor's go through a custom wrapper for creation
        CANSparkMax indexerMotor = new CANSparkMax( INDEXER.MOTOR_ID, MotorType.kBrushless );
        return new Indexer( indexerMotor );
    }

    @Override
    public void periodic () {

    }

    /**
    * We are overriding the initSendable and using it to send information back-and-forth between the robot program and
    * the user PC for live PID tuning purposes.
    * @param SendableBuilder This is inherited from SubsystemBase
    */ 
    @Override
    public void initSendable ( SendableBuilder builder ) {
        //builder.setSmartDashboardType( "Flywheel PID Tuning" );
        builder.addDoubleProperty( "P", this::GetP, this::SetP);
        builder.addDoubleProperty( "I", this::GetI, this::SetI);
        builder.addDoubleProperty( "D", this::GetD, this::SetD);
        builder.addDoubleProperty( "F", this::GetF, this::SetF);
        builder.addDoubleProperty( "Target (RPM)", this::GetTargetVelocity_RPM, this::SetTargetVelocity_RPM);
        builder.addBooleanProperty( "Closed-Loop On", this::IsClosedLoop, this::EnableClosedLoop );
    }

}

