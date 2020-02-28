package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret.TurretState_t;
import frc.robot.subsystems.Turret.ControlState_t;
import frc.robot.subsystems.Turret.FailingState_t;
import frc.robot.Constants.TURRET;
import frc.robot.lib.controllers.Vision.State;
import frc.robot.lib.controllers.Vision.Status;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import frc.robot.subsystems.Turret;
import frc.robot.lib.drivers.Limelight;
import frc.robot.lib.controllers.Vision;

// turn turret based off target command
public class TurretLimelight extends CommandBase {

    private boolean mIsFinished = true;
    private Status mStatus;
    private Limelight mLimelight; 
    private Vision mVision; 
    private final Logger mLogger = LoggerFactory.getLogger( TurretLimelight.class );
    private final Turret mTurret;
    private TurretState_t mTurretState;
    private ControlState_t mControlState;
    private FailingState_t mFailingState;
    private State mZeroing;
    private State mHolding;
    private State mLoading;
    private State mFail;

    /******************************************************************************************************************************
     ** COMMAND OVERRIDES
     ******************************************************************************************************************************/
    @Override
    public void initialize() {
        mLogger.info( "Starting TurretSpin command" );
        mIsFinished = false;

        // setInterruptible( false );

        mTurret.mVision.setState( State.kTurn );

        // mTurret.setState( mZeroing ); 

        // mVision.setLimelightState( mLimelight.ledMode.kOn ); 

        // mLimelight.setLimelightLEDOn();
    }

    @Override
    public void execute() {

        // Make sure the vision thread is processing the turning output
        if ( mTurret.mVision.getState() == State.kTurn ) {
            mStatus = mTurret.mVision.getStatus();
            // Check the status of the controller
            if ( mStatus == Status.kTargeting ) {
                mTurret.setOpenLoop( mTurret.mVision.getOutput() );
                // mLogger.info( "Target at: [{}]", mLimelight.horizontalToTargetDeg()); 
            } else if ( mStatus == Status.kLostTarget ) {
                // mIsFinished = true;
                mLogger.info( "Lost target" ); 
                // mLimelight.setLimelightLEDOff();
            } else if ( mStatus == Status.kReachedTarget ) {
                // mIsFinished = true;
                mLogger.info( "Reached target" );
            } else {
                mTurret.setOpenLoop(0.0);
                mLogger.warn( "Unknown status: [{}]", mStatus );
            }
        }

    
        // if ( mTurretState.getState() = mZeroing ) {
        //         mZeroing = mTurretState.getState();

        //     if ( mZeroing == mTurretState.ZeroingRetries ) {
        //         mZeroing.setState( mFailingState.ZeroingRetries );

        //     } else if ( mZeroing == mTurretState.ZeroingTimeout ) {
        //         mZeroing.setState( mFailingState.ZeroingTimeout );

        //     } else if ( mZeroing == mHolding ) {
        //         mZeroing.setState( mHolding );

        //     } else if ( mZeroing == mLoading ) {
        //         mZeroing.setState.mTurretState.Loading;

        //     } else {
        //         mTurret.mAlternateEncoder.setPosition();
        //     }
        // }

    }

    @Override
    public void end ( boolean interrupted ) {
        mLogger.info( "Finished TurnToTarget command" );
        mTurret.Stop(); 

        // mLimelight.setLimelightLEDOff(); 

    }

    @Override
    public boolean isFinished() {
        return false;
        // return mIsFinished; 
    }

    /******************************************************************************************************************************
     ** CONSTRUCTOR
     ******************************************************************************************************************************/
    public TurretLimelight ( Turret turret /* , Limelight limelight */ ) {
        mTurret = turret; 
        // mLimelight = limelight;
        addRequirements( mTurret ); 
        // addRequirements( mLimelight ); 
    }

}

