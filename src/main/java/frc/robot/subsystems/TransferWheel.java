package frc.robot.subsystems;

import frc.robot.Constants.TRANSFERWHEEL;
import frc.robot.lib.drivers.Photoeye;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import com.ctre.phoenix.motorcontrol.StickyFaults;

public class TransferWheel extends SubsystemBase {

    // Hardware
    private final CANSparkMax mTransferMotor;
    private final Photoeye mTransferPhotoeye = new Photoeye( TRANSFERWHEEL.TRANSFER_PHOTOEYE_ANALOG_CHANNEL );

    // Hardware states
    private boolean mIsBrakeMode;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( TransferWheel.class );

    public static enum TransferWheelState_t {
        Init { @Override public String toString() { return "Init"; } },
        Seeking { @Override public String toString() { return "Seeking"; } },
        Holding { @Override public String toString() { return "Holding"; } },
        Fail { @Override public String toString() { return "Fail"; } };
    }
    public static enum ControlState_t {
        ClosedLoop { @Override public String toString() { return "Closed-Loop"; } };
 public static enum FailingState_t {
        Healthy { @Override public String toString() { return "Healthy"; } },        
        BrokenSensor { @Override public String toString() { return "Broken sensor"; } },
        PowerLoss { @Override public String toString() { return "Power loss"; } },
        SeekTimeout { @Override public String toString() { return "Seek timeout"; } },
        SeekRetries { @Override public String toString() { return "Seek retries"; } };
    }
    private final CANSparkMax mMaster;

    private double mP, mI, mD, mF;
    private double mTargetVelocity_Units_Per_100ms;
    private double mTargetVelocity_RPM;
    private double mCurrentVelocity_RPM;
    private double mError_RPM;

    private double mTargetPercentOutput;

    // State variables
    private TransferWheelState_t mTransferWheelState;
    private ControlState_t mControlState;
    private FailingState_t mFailingState;
    private double mSeekTimer_S;
    private int mSeekRetries;

/**
    * @return TransferWheelState_t The current state of the TransferWheel.
    */
    public TransferWheelState_t GetTransferWheelState () {
        return mTransferWheelState;
    }

    /**
    * @return ControlState_t The current control state of the TransferWheel.
    */
    public ControlState_t GetControlState () {
        return mControlState;
    }

    /**
    * @return FailingState_t The current failing state of the TransferWheel.
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
    * @return double The current open-loop motor voltage percentage output.
    */
    public double GetTargetPercentOutput () {
        return mTargetPercentOutput;
    }

    private void Initialize () {
        mTransferMotor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative,
                                              TransferWheel.PID_IDX, GLOBAL.CAN_TIMEOUT_MS );
		mTransferMotor.setSensorPhase( true );
        mTransferMotor.setNeutralMode( SparkNeutralMode.Brake );
        mTransferWheelState = TransferWheelState_t.Init;
        mControlState = ControlState_t.ClosedLoop;
        mFailingState = FailingState_t.Healthy;
        mCurrentVelocity_RPM = Double.NaN;
        mError_RPM = 0;
        mP = TransferWheel.PID_KP;
        mI = TransferWheel.PID_KI;
        mD = TransferWheel.PID_KD;
        mF = TransferWheel.PID_KF;     
        SetTargetVelocity_RPM( 0.0 );  // 0's mSeekRetries
        SetTargetPercentOutput( 0.0 );
        SetGains();
        SetVelocityOutput();
    }
 
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
    private void SetF ( double f ) {
        mF = f;
    }

    /**
    * This method will get the feedforward gain of the velocity controller.
    */  
    private double GetF () {
        return mF;
    }

    private void SetGains () {
		mTransferMotor.config_kP( TransferWheel.PID_IDX, mP, GLOBAL.CAN_TIMEOUT_MS );
		mTransferMotor.config_kI( TransferWheel.PID_IDX, mI, GLOBAL.CAN_TIMEOUT_MS );
		mTransferMotor.config_kD( TransferWheel.PID_IDX, mD, GLOBAL.CAN_TIMEOUT_MS );
		mTransferMotor.config_kF( TransferWheel.PID_IDX, mF, GLOBAL.CAN_TIMEOUT_MS );
    }

    private boolean IsClosedLoop () {
        return mControlState == ControlState_t.ClosedLoop;
    }

    private void EnableClosedLoop ( boolean wantsClosedLoop ) {
        if (wantsClosedLoop && !IsClosedLoop() ) {
            SetGains();
            mControlState = ControlState_t.ClosedLoop;
        } else if ( !wantsClosedLoop && IsClosedLoop() ) {
            SetTargetPercentOutput( 0.0 );
            mControlState = ControlState_t.OpenLoop;
        }
    }

    private void SetSeekTimer () {
        mSeekTimer_S = Timer.getFPGATimestamp();
    }

    private boolean SensorIsBroken () {
        mTransferMotor.getStickyFaults( mTransferMotorFaults );
        return mTransferMotorFaults.SensorOverflow | mTransferMotorFaults.SensorOutOfPhase | mTransferMotorFaults.RemoteLossOfSignal;
    }

    private boolean PowerDisruption () {
        mMaster.getStickyFaults( mMasterFaults );
        mFollower.getStickyFaults( mFollowerFaults );
        return mMasterFaults.ResetDuringEn | mMasterFaults.UnderVoltage | mFollowerFaults.ResetDuringEn | mFollowerFaults.UnderVoltage;
    }

    /**
    * This method will set output mode of the TalonSRX's to velocity at a target defined by the variable
    * mTargetVelocity_Units_Per_100ms.
    */    
    private void SetVelocityOutput () {
        mMaster.set( ControlMode.Velocity, mTargetVelocity_Units_Per_100ms );
    }

    /**
    * This method will set output mode of the TalonSRX's to percentage at a target defined by the variable
    * mTargetPercentOutput.
    */  
    private void SetPercentOutput () {
        mMaster.set( ControlMode.PercentOutput, mTargetPercentOutput );
    }

    /**
    * This method will set the TalonSRX output based on the control state.
    */
    private void MotorOutput () {  
        switch ( mControlState ) {
            case OpenLoop:
                SetPercentOutput(); 
                break;
            case ClosedLoop:
                SetVelocityOutput();
                break;
        }
    }

    public void Update () {
        switch ( mTransfereWheelState ) {
            case Init:
                // No velocity command has come in, so just hold at 0.0
                if ( mTargetVelocity_Units_Per_100ms == 0.0 ) {
                    mTransfereWheelState = TransfereWheelState_t.Holding;
                
                // Set the seek timer and begin seeking to the velocity target
                } else  {
                    mTransfereWheelState = TransfereWheelState_t.Seeking;
                    SetSeekTimer();
                }
                break;

            case Seeking:
                // The velocity is within tolerance to move to the holding state...which is a go for shooting
                if ( Math.abs( mError_RPM ) <= TransfereWheel.OFFTRACK_ERROR_PERCENT * mTargetVelocity_RPM ) {
                    mTransfereWheelState = TransfereWheelState_t.Holding;
                }
                // The seek timer has expired
                if ( Timer.getFPGATimestamp() - mSeekTimer_S > TransferWheel.SEEK_TIMER_EXPIRED_S ) {
                    mTransferWheelState = TransferWheelState_t.Fail;
                    mSeekRetries += 1;
                    // The retries have been exhausted
                    if ( mSeekRetries > TransferWheel.SEEK_RETRY_LIMIT ) {
                        mControlState = ControlState_t.OpenLoop;
                        mFailingState = FailingState_t.SeekRetries;
                    } else {
                        mFailingState = FailingState_t.SeekTimeout;
                    }
                }
                break;

            case Holding:
                // The velocity has fallen outside of tolerance, move back to the seeking state
                if ( Math.abs( mError_RPM ) > TransferWheel.OFFTRACK_ERROR_PERCENT * mTargetVelocity_RPM ) {
                    mTransferWheelState = TransferWheelState_t.Seeking;
                    SetSeekTimer();
                }
                break;

            case Fail:
                switch ( mFailingState ) {
                    // Failure due to broken sensor, keep checking if it comes back online
                    case BrokenSensor:
                        mMaster.clearStickyFaults( 0 ); // Send command whithout checking or blocking
                        if ( !SensorIsBroken() ) {
                            Initialize();
                        }
                        break;

                    case PowerLoss:
                        mMaster.clearStickyFaults( 0 ); // Send command whithout checking or blocking
                        mFollower.clearStickyFaults( 0 ); // Send command whithout checking or blocking
                        if ( !PowerDisruption() ) {
                            Initialize();
                        }
                        break;

                    // Failure due to timing out during a seek
                    case SeekTimeout:
                        mFailingState = FailingState_t.Healthy;
                        mTransferWheelState = TransferWheelState_t.Seeking;
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

        if ( SensorIsBroken() ) {
            mControlState = ControlState_t.OpenLoop;
            mTransferWheelState = TransferWheelState_t.Fail;
            mFailingState = FailingState_t.BrokenSensor;

        // The motor control lost power and/or reset    
        } else if ( PowerDisruption() ) {
            mControlState = ControlState_t.OpenLoop;
            mTransferWheelState = TransferWheelState_t.Fail;
            mFailingState = FailingState_t.PowerLoss;                    
        }

        // Set the ouptut based on the control state
        MotorOutput();
    }

    public TransferWheel ( WPI_TalonSRX master, StickyFaults masterFaults,
                      WPI_TalonSRX follower, StickyFaults followerFaults ) {
        mMaster = master;
        mFollower = follower;
        mMasterFaults = masterFaults;
        mFollowerFaults = followerFaults;
        Initialize();
    }

    @Override
    public void periodic () {
        mCurrentVelocity_RPM = mMaster.getSelectedSensorVelocity(TRANSFERWHEEL.PID_IDX ) /
                                                                  TRANSFERWHEEL.SENSOR_UNITS_PER_ROTATION * 4096;
        mError_RPM = GetTargetVelocity_RPM() - mCurrentVelocity_RPM;     
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






    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        if (wantsBrakeMode && !mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mTransferMotor.setIdleMode(IdleMode.kBrake);
      mLogger.info("Neutral mode set to: [Brake]");

    } else if (!wantsBrakeMode && mIsBrakeMode) {
      mIsBrakeMode = wantsBrakeMode;
      mTransferMotor.setIdleMode(IdleMode.kCoast);
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

    public TransferWheel ( CANSparkMax transferMotor ) {
  
        // Set the hardware
        mTransferMotor = transferMotor;

        // Set the hardware states
        mIsBrakeMode = false;
        SetBrakeMode( true );
    }

    public static TransferWheel create () {
        // Talon's and Victor's go through a custom wrapper for creation
        CANSparkMax transferMotor = new CANSparkMax( Constants.TRANSFER_MOTOR_ID, MotorType.kBrushless );
        return new TransferWheel( transferMotor );
        StickyFaults TransferWheelFaults = new StickyFaults();
    }

    

}
