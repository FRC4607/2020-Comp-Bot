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
       
    }

    // Hardware
    private final CANSparkMax mIndexerMotor;
    private final Photoeye mIndexerPhotoeye = new Photoeye( INDEXER.PHOTOEYE_ANALOG_CHANNEL );

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
        //mIndexerMotor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative,
                                            //  INDEXER.PID_IDX, GLOBAL.CAN_TIMEOUT_MS );
		//mIndexerMotor.setSensorPhase( true );
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
    * This method will set output mode of the TalonSRX's to velocity at a target defined by the variable
    * mTargetVelocity_Units_Per_100ms.
    */    
    private void SetVelocityOutput () {
        mIndexerMotor.set( ControlState.Velocity, mTargetVelocity_Units_Per_100ms );
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

}

