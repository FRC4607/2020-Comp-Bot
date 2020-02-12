package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.lib.drivers.Photoeye;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TransferWheel extends SubsystemBase {

    // Hardware
    private final CANSparkMax mTransferMotor;
    private final Photoeye mTransferPhotoeye = new Photoeye( Constants.TRANSFER_PHOTOEYE_ANALOG_CHANNEL );

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
    }

    @Override
    public void periodic () {

    }

}
