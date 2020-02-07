package frc.robot.subsystems;

import frc.robot.Constants.HOOD;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class Hood extends SubsystemBase {

    // State enumerations
    public static enum HoodState_t {
        Init { @Override public String toString() { return "Init"; } },
        Seeking { @Override public String toString() { return "Seeking"; } },
        Holding { @Override public String toString() { return "Holding"; } },
        Fail { @Override public String toString() { return "Fail"; } };
    }
    
    public static enum ControlState_t {
        OpenLoop { @Override public String toString() { return "Open-Loop"; } },
        ClosedLoop { @Override public String toString() { return "Closed-Loop"; } };
    }

    // Hardware
    private final CANSparkMax mMaster;
    private CANEncoder mAlternateEncoder;
    private CANPIDController mPIDController;

    // Closed-loop control
    private double mP, mI, mD, mF;
    private double mTargetPosition_Deg;
    private double mCurrentPosition_Deg;
    //private double mError_DEG;

    // State variables
    private HoodState_t mHoodState;    
    private ControlState_t mControlState;
    
    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------
    
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
    * @return double The current closed-loop position target in degrees.
    */
    public double GetTargetPosition_Deg () {
        return mTargetPosition_Deg;
    }

    /**
    * @param double The target position in degrees for closed-loop position control.
    */
    public void SetTargetPosition_Deg ( double targetPosition_Deg ) {
        mTargetPosition_Deg = targetPosition_Deg;
        // Likely need to convert the target position into the SparkMax units
    }

    /**
    * @return double The current current velocity of the flywheel in RPM.
    */
    public double GetCurrentVelocity_RPM () {
        return mCurrentPosition_Deg;
    }


    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    /**
    * This method will initialize the Hood subsystem by setting the up the master SparkMax for position control and
    * setting all of the internal states to their starting values.
    */
    private void Initialize () {
        mMaster.setIdleMode( IdleMode.kBrake );
        mMaster.setInverted( true );
        mPIDController.setFeedbackDevice( mAlternateEncoder );
        mAlternateEncoder.setInverted( false );
        mHoodState = HoodState_t.Init;
        mControlState = ControlState_t.OpenLoop;
        mP = HOOD.PID_KP;
        mI = HOOD.PID_KI;
        mD = HOOD.PID_KD;
        mF = HOOD.PID_KF;     

        // TODO: add a zero'ing algorithm for the position sensor
        SetTargetPosition_Deg( mAlternateEncoder.getPosition() );  

        SetGains();
        SetPercentOutput();

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
    * This method will set output mode of the SparkMax to a 0.0 percentage.  This is the default mode when there is an
    * error in the position closed-loop.
    */  
    private void SetPercentOutput () {
        mMaster.set( 0.0 );
    }    

    /**
    * This method will set the velocity closed-loop gains of the TalonSRX.
    */
    private void SetGains () {
        mPIDController.setP( mP, HOOD.PID_IDX );
        mPIDController.setP( mI, HOOD.PID_IDX );
        mPIDController.setP( mI, HOOD.PID_IDX );
        mPIDController.setP( mF, HOOD.PID_IDX );
    }

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    /**
    * This is the flywheel class consructor.  It is setup for injecting the dependencies in order to allow for mocking
    * those dependencies during unit-testing.
    *
    * @param master CANSparkMax master motor controller
    * @param masterFaults StickyFaults for the master
    * @param follower WPI_TalonSRX follower motor controller
    * @param followerFaults StickyFaults for the follower
    */
    public Hood ( CANSparkMax master, CANPIDController pidController, CANEncoder alternateEncoder ) {
        mMaster = master;
        mAlternateEncoder = alternateEncoder;
        mPIDController = pidController;
        Initialize();
    }

    /**
    * This is methods calls the hood consructor and creates the hood object.  The motor controllers objects are sent
    * through a custom wrapper to ensure each motor controller created is in a known state.
    * @see {@link frc.robot.lib.drivers.SparkMax}
    */   
    public static Hood create () {
        CANSparkMax master =  SparkMax.CreateSparkMax( new CANSparkMax( HOOD.MASTER_ID, MotorType.kBrushless ) );
        CANEncoder alternateEncoder = master.getAlternateEncoder( AlternateEncoderType.kQuadrature, HOOD.SENSOR_COUNTS_PER_ROTATION );
        CANPIDController pidController = master.getPIDController();
        return new Hood( master, pidController, alternateEncoder );
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
        //builder.addDoubleProperty( "Target (RPM)", this::GetTargetVelocity_RPM, this::SetTargetVelocity_RPM);
        //builder.addBooleanProperty( "Closed-Loop On", this::IsClosedLoop, this::EnableClosedLoop );
    }



}




//     public void SmartMotion {
//         if (mControlState != controlType.kSmartMotion) {
//             mControlState = controlType.kSmartMotion;
//             mHoodMotor.selectProfileSlot(Constants.SMART_MOTION_ID);
//         }

//         mEncoderPositionTicks = getEncoderPositionTicks();
        
//         if (mEncoderPositionTicks < targetPositionTicks) {
//             mHoodFFGravityComponent = Math.cos(Math.toRadians(getAngle(mEncoderPositionTicks)));
//         } else {
//             mHoodFFGravityComponent = 0.0;
//     }
//     // mLogger.info("Encoder position: {}, target position: {}, Feed Forward: {}", mEncoderPositionTicks, targetPositionTicks, mWristFFGravityComponent);
//     mHoodMotor.set(ControlType.IntState, targetPositionTicks);
//   }