package frc.robot.subsystems;

import frc.robot.Constants.HOOD;
import frc.robot.Constants.CURRENT_LIMIT;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.LinearFilter;

public class Hood extends SubsystemBase {

    // State enumerations
    public static enum HoodState_t {
        Init { @Override public String toString() { return "Init"; } },
        Zeroing { @Override public String toString() { return "Zeroing"; } },
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
        ZeroingTimeout { @Override public String toString() { return "Zeroing timeout"; } },
        ZeroingRetries { @Override public String toString() { return "Zeroing retries"; } };
    }

    // Hardware
    private CANSparkMax mMaster;
    private CANEncoder mAlternateEncoder;
    private CANPIDController mPIDController;

    // Closed-loop control
    private double mP, mI, mD, mF, mMaxVelocity, mMaxAcceleration;
    private double mTargetPosition_Rot;
    private double mCurrentPosition_Rot;
    private double mSmartCurrentLimit;
    //private double mError_Rot;

    // Open-loop control
    private double mTargetPercentOutput;

    // Zero'ing the encoder
    private LinearFilter mMovingAverageFilter;
    private double mCurrentPosition_Rot_Filtered;
    private double mZeroingTimer_S;
    private int mZeroingRetries;

    // State variables
    private HoodState_t mHoodState;    
    private ControlState_t mControlState;
    private FailingState_t mFailingState;
    
    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------
    
    // // Logging
    // private final Logger mLogger = LoggerFactory.getLogger( Drivetrain.class );

    // public void SetBrakeMode ( boolean wantsBrakeMode ) {
    //     if (wantsBrakeMode && !mIsBrakeMode) {
    //         mIsBrakeMode = wantsBrakeMode;
    //         mMaster.setIdleMode( IdleMode.Brake );
    //         mLogger.info( "Neutral mode set to: [Brake]" );

    //     } else if (!wantsBrakeMode && mIsBrakeMode) {
    //         mIsBrakeMode = wantsBrakeMode;
    //         // mMaster.setIdleMode( IdleMode.Coast );
    //         mLogger.info( "Neutral mode set to: [Coast]" );
    //     }
    // }


    // open loop drive
    public void setOpenLoop (double xHood) {
        mMaster.set( xHood );
    }

    // stop for deadband
    public void Stop() {
        mMaster.set( 0.0 );
    }

    /**
    * @return HoodState_t The current state of the hood.
    */
    public HoodState_t GetHoodState () {
        return mHoodState;
    }

    /**
    * @return ControlState_t The current control state of the hood.
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
    * @return double The current closed-loop position target in rotations.
    */
    public double GetTargetPosition_Rot () {
        return mTargetPosition_Rot;
    }

    /**
    * @param double The target position in rotations for closed-loop position control.
    */
    public void SetTargetPosition_Rot ( double targetPosition_Rot ) {
        mTargetPosition_Rot = targetPosition_Rot;
    }

    /**
    * @return double The current current position of the hood in rotations.
    */
    public double GetCurrentPosition_Rot () {
        return mCurrentPosition_Rot;
    }

    /**
    * @return double The current open-loop motor voltage percentage output.
    */
    public double GetTargetPercentOutput () {
        return mTargetPercentOutput;
    }    


    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    /**
    * This method will initialize the Hood subsystem by setting the internal states to their starting values and
    * intialize the closed loop gains/targets.  
    */
    private void Initialize () {
        mPIDController.setFeedbackDevice( mAlternateEncoder );
        mAlternateEncoder.setInverted( false );
        mTargetPosition_Rot = 0.0;
        mTargetPercentOutput = 0.0;
        mZeroingTimer_S = 0.0;
        mZeroingRetries = 0;
        mCurrentPosition_Rot = Double.NaN;
        mCurrentPosition_Rot_Filtered = Double.NaN;
        mHoodState = HoodState_t.Init;
        mControlState = ControlState_t.OpenLoop;
        mFailingState = FailingState_t.Healthy;
        mP = HOOD.PID_KP;
        mI = HOOD.PID_KI;
        mD = HOOD.PID_KD;
        mF = HOOD.PID_KF;
        mMaxVelocity = HOOD.MAX_VELOCITY;
        mMaxAcceleration = HOOD.MAX_ACCELERATION;     
        SetGains();
    }

    /**
    * This method will set the P-gain of the position controller.
    */  
    private void SetP ( double p ) {
        mP = p;
    }
    
    /**
    * This method will get the P-gain of the position controller.
    */  
    private double GetP () {
        return mP;
    }

    /**
    * This method will set the I-gain of the position controller.
    */  
    private void SetI ( double i ) {
        mI = i;
    }

    /**
    * This method will get the I-gain of the position controller.
    */  
    private double GetI () {
        return mI;
    }

    /**
    * This method will set the D-gain of the position controller.
    */  
    private void SetD ( double d ) {
        mD = d;
    }

    /**
    * This method will get the D-gain of the position controller.
    */  
    private double GetD () {
        return mD;
    }

    /**
    * This method will set the feedforward gain of the position controller.
    */  
    private void SetF ( double f ) {
        mF = f;
    }

    /**
    * This method will get the feedforward gain of the position controller.
    */  
    private double GetF () {
        return mF;
    }

    /**
    * This method will set the max velocty used by the position controller.
    */  
    private void SetMaxVelocity ( double maxVelocity ) {
        mMaxVelocity = maxVelocity;
    }

    /**
    * This method will get the max velocty used by the position controller.
    */  
    private double GetMaxVelocity () {
        return mMaxVelocity;
    }

    /**
    * This method will set the max acceleration used by the position controller.
    */  
    private void SetMaxAcceleration ( double maxAcceleration ) {
        mMaxAcceleration = maxAcceleration;
    }

    /**
    * This method will get the max acceleration used by the position controller.
    */  
    private double GetMaxAcceleration () {
        return mMaxAcceleration;
    }

    /**
    * This method will set output mode of the SparkMax to Smart Motion closed-loop control.
    */  
    private void SetSmartMotionOutput () {
        mPIDController.setReference( mTargetPosition_Rot, ControlType.kSmartMotion );
    }

    /**
    * This method will set output mode of the SparkMax to a percentage defined by the variable 
    * mTargetPercentOutput.  The only time this variable isn't 0.0 is when the hood is zero'ing the encoder
    * position.
    */  
    private void SetPercentOutput () {
        mMaster.set( mTargetPercentOutput );
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
                SetSmartMotionOutput();
                break;
        }
    }

    /**
    * This method will set the velocity closed-loop gains of the Spark Max.
    */
    private void SetGains () {
        mPIDController.setP( mP, HOOD.PID_IDX );
        mPIDController.setI( mI, HOOD.PID_IDX );
        mPIDController.setD( mD, HOOD.PID_IDX );
        mPIDController.setFF( mF, HOOD.PID_IDX );
        mPIDController.setIZone( 0.0, HOOD.PID_IDX );
        mPIDController.setOutputRange( -1.0, 1.0 );
        mPIDController.setSmartMotionMaxVelocity( mMaxVelocity, HOOD.PID_IDX );
        mPIDController.setSmartMotionMinOutputVelocity( 0.0, HOOD.PID_IDX );
        mPIDController.setSmartMotionMaxAccel( mMaxAcceleration, HOOD.PID_IDX );
        mPIDController.setSmartMotionAllowedClosedLoopError( 0.1, HOOD.PID_IDX );
    }

    /**
    * This method will zero the hood by driving the hood slowly to the hard-stop and monitor the position as the
    * indicator.  It is expected that the robot is starting with the hood already against the hard-stop, but in the
    * case of a powerloss/reset or some other fault, the system will be able to re-zero and continue using
    * closed-loop position control.
    * <p>
    * Due to how the hood state machine is setup, the motor is guarenteed to be driven for 1 loop period (20ms) before
    * checking the position.
    */
    private boolean IsZeroingDone () {
        if ( !Double.isNaN( mCurrentPosition_Rot ) && !Double.isNaN( mCurrentPosition_Rot_Filtered ) ) {
            if ( Math.abs( mCurrentPosition_Rot - mCurrentPosition_Rot_Filtered ) < HOOD.ZEROING_DONE_THRESHOLD ) {
                return true;
            }
        }
        return false;
    }

    /**
    * This method will update the states of the variable mHoodState, mControlState, and mFailingState.  After
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
        switch ( mHoodState ) {
            case Init:
                // Always move to Zeroing state when we arrive at the Init state, start trying to move the hood slowly
                mTargetPercentOutput = HOOD.ZEROING_MOTOR_OUTPUT;
                // mSmartCurrentLimit = CURRENT_LIMIT.RPM_LIMIT;
                mZeroingTimer_S = Timer.getFPGATimestamp(); 
                mHoodState = HoodState_t.Zeroing;
                break;

            case Zeroing:
                // The zeroing timer has expired
                if ( Timer.getFPGATimestamp() - mZeroingTimer_S > HOOD.ZEROING_TIMER_EXPIRED_S ) {
                    mTargetPercentOutput = 0.0;
                    mZeroingRetries += 1;
                    // The retries have been exhausted
                    if ( mZeroingRetries > HOOD.ZEROING_RETRY_LIMIT ) {
                        mFailingState = FailingState_t.ZeroingRetries;
                    } else {
                        mFailingState = FailingState_t.ZeroingTimeout;
                    }
                }
                // The hood has reached the hard-stop, zero the position and move on
                else if ( IsZeroingDone() ) {
                    mAlternateEncoder.setPosition( mCurrentPosition_Rot );
                    // There's no request to seek another hood position, hold this position
                    if ( mTargetPosition_Rot == 0.0 ) {
                        mHoodState = HoodState_t.Holding;
                    // Request to seek
                    } else {
                        mHoodState = HoodState_t.Seeking;
                    }
                }
                break;

            case Seeking:
                break;

            case Holding:
                break;

            case Fail:
                switch ( mFailingState ) {
                    // Failure due to timing out during zero'ing, retry with 10% more cowbell
                    case ZeroingTimeout:
                        mTargetPercentOutput = HOOD.ZEROING_MOTOR_OUTPUT * 1.1;  
                        mZeroingTimer_S = Timer.getFPGATimestamp();
                        mHoodState = HoodState_t.Zeroing;
                        break;

                    case ZeroingRetries:
                        break;

                    default:
                        break;
                }
                break;

        }       
            
            // // No velocity command has come in, so just hold at 0.0
            //     if ( mTargetVelocity_Units_Per_100ms == 0.0 ) {
                    
                
            //     // Set the seek timer and begin seeking to the velocity target
            //     } else  {
            //         mFlywheelState = FlywheelState_t.Seeking;
            //         SetSeekTimer();
            //     }
            //     break;

        // The feedback sensor isn't working

        // The motor control lost power and/or reset    

        // Set the ouptut based on the control state
        MotorOutput();

    }

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    /**
    * This is the hood class consructor.  It is setup for injecting the dependencies in order to allow for mocking
    * those dependencies during unit-testing.
    *
    * @param master CANSparkMax master motor controller
    * @param pidController CANPIDController PID controller used for position control
    * @param alternateEncoder CANEncoder alternate encoder used for position control (REV Through Bore)
    */
    public Hood ( CANSparkMax master, CANPIDController pidController, CANEncoder alternateEncoder, LinearFilter movingAverageFilter ) {
        mMaster = master;
        mAlternateEncoder = alternateEncoder;
        mPIDController = pidController;
        mMovingAverageFilter = movingAverageFilter;
        Initialize();
        // Current limiting
        // mMaster.setSmartCurrentLimit( 6, 6, CURRENT_LIMIT.RPM_LIMIT );
    }

    /**
    * This is methods calls the hood consructor and creates the hood object.  The motor controllers objects are sent
    * through a custom wrapper to ensure each motor controller created is in a known state.  The alternate encoder is
    * a REV Through bore and is used for posion control feedback.  The PID controller is used for all positional
    * control.
    * @see {@link frc.robot.lib.drivers.SparkMax}
    */   
    public static Hood create () {
        CANSparkMax master =  SparkMax.CreateSparkMax( new CANSparkMax( HOOD.MASTER_ID, MotorType.kBrushless ) );
        CANEncoder alternateEncoder = master.getAlternateEncoder( AlternateEncoderType.kQuadrature, HOOD.SENSOR_COUNTS_PER_ROTATION );
        CANPIDController pidController = master.getPIDController();
        LinearFilter movingAverageFilter = LinearFilter.movingAverage( HOOD.MOVING_AVERAGE_FILTER_TAPS );
        return new Hood( master, pidController, alternateEncoder, movingAverageFilter );
    }

    /**
    * The subsystem periodic method gets called by the CommandScheduler at the very beginning of each robot loop.  All
    * sensor readings should be updated in this method.
    * @see {@link edu.wpi.first.wpilibj2.command.CommandScheduler#run}
    */ 
    @Override
    public void periodic () {
        mCurrentPosition_Rot = mAlternateEncoder.getPosition();
        mCurrentPosition_Rot_Filtered = mMovingAverageFilter.calculate( mCurrentPosition_Rot );
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
        builder.addDoubleProperty( "MaxVel", this::GetMaxVelocity, this::SetMaxVelocity);
        builder.addDoubleProperty( "MaxAcc", this::GetMaxAcceleration, this::SetMaxAcceleration);
        //builder.addDoubleProperty( "Target (RPM)", this::GetTargetVelocity_RPM, this::SetTargetVelocity_RPM);
        //builder.addBooleanProperty( "Closed-Loop On", this::IsClosedLoop, this::EnableClosedLoop );
    }



}
