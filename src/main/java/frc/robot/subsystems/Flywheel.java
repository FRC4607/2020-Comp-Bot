package frc.robot.subsystems;

import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.FLYWHEEL;
import frc.robot.lib.drivers.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import java.lang.Math;

public class Flywheel extends SubsystemBase {

    // State enumerations
    public static enum FlywheelState_t {
        Init { @Override public String toString() { return "Init"; } },
        Seeking { @Override public String toString() { return "Seeking"; } },
        Holding { @Override public String toString() { return "Holding"; } },
        Fail { @Override public String toString() { return "Fail"; } };
    }

    public static enum ControlState_t {
        OpenLoop { @Override public String toString() { return "Open-Loop"; } },
        ClosedLoop { @Override public String toString() { return "Closed-Loop"; } };
    }

    public static enum FailingState_t {
        Healthy { @Override public String toString() { return "Healthy"; } },        
        BrokenSensor { @Override public String toString() { return "Broken sensor"; } },
        PowerLoss { @Override public String toString() { return "Power loss"; } },
        SeekTimeout { @Override public String toString() { return "Seek timeout"; } },
        SeekRetries { @Override public String toString() { return "Seek retries"; } };
    }

    // Hardware
    private final WPI_TalonSRX mMaster;
    private final WPI_TalonSRX mFollower;
    private StickyFaults mMasterFaults;
    private StickyFaults mFollowerFaults;

    // Closed-loop control
    private double mP, mI, mD, mF;
    private double mTargetVelocity_Units_Per_100ms;
    private double mTargetVelocity_RPM;
    private double mCurrentVelocity_RPM;
    private double mError_RPM;

    // Open-loop control
    private double mTargetPercentOutput;

    // State variables
    private FlywheelState_t mFlywheelState;
    private ControlState_t mControlState;
    private FailingState_t mFailingState;
    private double mSeekTimer_S;
    private int mSeekRetries;

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    /**
    * @return FlywheelState_t The current state of the flywheel.
    */
    public FlywheelState_t GetFlywheelState () {
        return mFlywheelState;
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
    * @return double The current open-loop motor voltage percentage output.
    */
    public double GetTargetPercentOutput () {
        return mTargetPercentOutput;
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
        mTargetVelocity_Units_Per_100ms = targetVelocity_RPM * FLYWHEEL.SENSOR_UNITS_PER_ROTATION / 600.0;
    }

    /**
    * @param double The target motor voltage percentage output for open-loop control.
    */    
    public void SetTargetPercentOutput ( double targetPercentOutput ) {
        mTargetPercentOutput = targetPercentOutput;
    }

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------    

    /**
    * This method will initialize the Flywheel subsystem by setting the up the
    * master TalonSRX for velocity control and setting all of the internal
    * states to their starting values.
    */
    private void Initialize () {
        mMaster.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative,
                                              FLYWHEEL.PID_IDX, GLOBAL.CAN_TIMEOUT_MS );
		mMaster.setSensorPhase( true );
        mMaster.setNeutralMode( NeutralMode.Brake );
        mFollower.setNeutralMode( NeutralMode.Brake );
        mFollower.setInverted( InvertType.OpposeMaster );
        mFlywheelState = FlywheelState_t.Init;
        mControlState = ControlState_t.ClosedLoop;
        mFailingState = FailingState_t.Healthy;
        mCurrentVelocity_RPM = Double.NaN;
        mError_RPM = 0;
        mP = FLYWHEEL.PID_KP;
        mI = FLYWHEEL.PID_KI;
        mD = FLYWHEEL.PID_KD;
        mF = FLYWHEEL.PID_KF;     
        SetTargetVelocity_RPM( 0.0 );  // 0's mSeekRetries
        SetTargetPercentOutput( 0.0 );
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
    private void SetF ( double f ) {
        mF = f;
    }

    /**
    * This method will get the feedforward gain of the velocity controller.
    */  
    private double GetF () {
        return mF;
    }

    /**
    * This method will set the velocity closed-loop gains of the TalonSRX.
    */
    private void SetGains () {
		mMaster.config_kP( FLYWHEEL.PID_IDX, mP, GLOBAL.CAN_TIMEOUT_MS );
		mMaster.config_kI( FLYWHEEL.PID_IDX, mI, GLOBAL.CAN_TIMEOUT_MS );
		mMaster.config_kD( FLYWHEEL.PID_IDX, mD, GLOBAL.CAN_TIMEOUT_MS );
		mMaster.config_kF( FLYWHEEL.PID_IDX, mF, GLOBAL.CAN_TIMEOUT_MS );
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
        if (wantsClosedLoop && !IsClosedLoop() ) {
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
        mMaster.getStickyFaults( mMasterFaults );
        return mMasterFaults.SensorOverflow | mMasterFaults.SensorOutOfPhase | mMasterFaults.RemoteLossOfSignal;
    }

    /**
    * This method will check the sticky faults of the master and follower TalonSRX for faults related to the power
    * supplied. 
    * @return boolean Logical OR of ResetDuringEn and UnderVoltage for both TalonSRX's
    */    
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
        switch ( mFlywheelState ) {
            case Init:
                // No velocity command has come in, so just hold at 0.0
                if ( mTargetVelocity_Units_Per_100ms == 0.0 ) {
                    mFlywheelState = FlywheelState_t.Holding;
                
                // Set the seek timer and begin seeking to the velocity target
                } else  {
                    mFlywheelState = FlywheelState_t.Seeking;
                    SetSeekTimer();
                }
                break;

            case Seeking:
                // The velocity is within tolerance to move to the holding state...which is a go for shooting
                if ( Math.abs( mError_RPM ) <= FLYWHEEL.OFFTRACK_ERROR_PERCENT * mTargetVelocity_RPM ) {
                    mFlywheelState = FlywheelState_t.Holding;
                }
                // The seek timer has expired
                if ( Timer.getFPGATimestamp() - mSeekTimer_S > FLYWHEEL.SEEK_TIMER_EXPIRED_S ) {
                    mFlywheelState = FlywheelState_t.Fail;
                    mSeekRetries += 1;
                    // The retries have been exhausted
                    if ( mSeekRetries > FLYWHEEL.SEEK_RETRY_LIMIT ) {
                        mControlState = ControlState_t.OpenLoop;
                        mFailingState = FailingState_t.SeekRetries;
                    } else {
                        mFailingState = FailingState_t.SeekTimeout;
                    }
                }
                break;

            case Holding:
                // The velocity has fallen outside of tolerance, move back to the seeking state
                if ( Math.abs( mError_RPM ) > FLYWHEEL.OFFTRACK_ERROR_PERCENT * mTargetVelocity_RPM ) {
                    mFlywheelState = FlywheelState_t.Seeking;
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
                        mFlywheelState = FlywheelState_t.Seeking;
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

        // The feedback sensor isn't working
        if ( SensorIsBroken() ) {
            mControlState = ControlState_t.OpenLoop;
            mFlywheelState = FlywheelState_t.Fail;
            mFailingState = FailingState_t.BrokenSensor;

        // The motor control lost power and/or reset    
        } else if ( PowerDisruption() ) {
            mControlState = ControlState_t.OpenLoop;
            mFlywheelState = FlywheelState_t.Fail;
            mFailingState = FailingState_t.PowerLoss;                    
        }

        // Set the ouptut based on the control state
        MotorOutput();
    }

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    /**
    * This is the flywheel class consructor.  It is setup for injecting the dependencies in order to allow for mocking
    * those dependencies during unit-testing.
    *
    * @param master WPI_TalonSRX master motor controller
    * @param masterFaults StickyFaults for the master
    * @param follower WPI_TalonSRX follower motor controller
    * @param followerFaults StickyFaults for the follower
    */
    public Flywheel ( WPI_TalonSRX master, StickyFaults masterFaults,
                      WPI_TalonSRX follower, StickyFaults followerFaults ) {
        mMaster = master;
        mFollower = follower;
        mMasterFaults = masterFaults;
        mFollowerFaults = followerFaults;
        Initialize();
    }

    /**
    * This is methods calls the flywheel consructor and creates the flywheel object.  The motor controllers objects
    * are sent through a custom wrapper to ensure each motor controller created is in a known state.
    * @see {@link frc.robot.lib.drivers.TalonSRX}
    */    
    public static Flywheel create () {
        WPI_TalonSRX master = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( FLYWHEEL.MASTER_ID) );
        WPI_TalonSRX follower = TalonSRX.createTalonSRX( new WPI_TalonSRX( FLYWHEEL.FOLLOWER_ID), master );
        StickyFaults masterFaults = new StickyFaults();
        StickyFaults followerFaults = new StickyFaults();

        return new Flywheel( master, masterFaults, follower, followerFaults );
    }

    /**
    * The subsystem periodic method gets called by the CommandScheduler at the very beginning of each robot loop.  All
    * sensor readings should be updated in this method.
    * @see {@link edu.wpi.first.wpilibj2.command.CommandScheduler#run}
    */ 
    @Override
    public void periodic () {
        mCurrentVelocity_RPM = mMaster.getSelectedSensorVelocity( FLYWHEEL.PID_IDX ) /
                                                                  FLYWHEEL.SENSOR_UNITS_PER_ROTATION * 600.0;
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
