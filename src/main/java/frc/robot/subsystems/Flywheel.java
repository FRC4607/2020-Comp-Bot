package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Flywheel.FlywheelJoystick;

// Creates the elevator subsystem
public class Flywheel extends SubsystemBase {

  public static WPI_TalonSRX mRightLeader;
  public static WPI_TalonSRX mLeftFollow; 

  // public static SpeedControllerGroup shooterDrive = new SpeedControllerGroup(mRightLeader);

  // Hardware states
  private boolean mIsBrakeMode;
  private boolean mIsInverted;

  // Logger
  private final Logger mLogger = LoggerFactory.getLogger(Flywheel.class);

  /****************************************************************************************************************************** 
  ** GETTERS
  ******************************************************************************************************************************/
  public boolean isBrakeMode() {
    return mIsBrakeMode;
  }

  public boolean isInverted() {
    return mIsInverted;
  }

  /****************************************************************************************************************************** 
  ** INVERT MOTOR OUTPUT
  ******************************************************************************************************************************/
  public void InvertOutput(boolean invert) {
    if (invert != mIsInverted) {
      mIsInverted = invert;
      mRightLeader.setInverted(invert);
      mLeftFollow.setInverted(invert);
      mLogger.info("Set inverted: [{}]", mIsInverted);
    }
  }

  /****************************************************************************************************************************** 
  ** SET BRAKE/COAST MODE
  ******************************************************************************************************************************/
  public void setBrakeMode(boolean wantsBrakeMode) {
    if (wantsBrakeMode && !mIsBrakeMode) {
      mIsBrakeMode = wantsBrakeMode;
      mRightLeader.setNeutralMode(NeutralMode.Brake);
      mLeftFollow.setNeutralMode(NeutralMode.Brake);
      mLogger.info("Neutral mode set to: [Brake]");

    } else if (!wantsBrakeMode && mIsBrakeMode) {
      mIsBrakeMode = wantsBrakeMode;
      mRightLeader.setNeutralMode(NeutralMode.Coast);
      mLeftFollow.setNeutralMode(NeutralMode.Coast);
      mLogger.info("Neutral mode set to: [Coast]");
    }
  }

  /****************************************************************************************************************************** 
  ** APPLY MOTOR OUTPUT
  ******************************************************************************************************************************/
  // The onus is on the caller to ensure that the sum of the drive signal is between -1.0 and 1.0
  public void ApplyDriveSignal(double throttle) {
    double mThrottle = throttle;
    mRightLeader.set(mThrottle);
  }

  /****************************************************************************************************************************** 
  ** SET OPEN-LOOP MODE
  ******************************************************************************************************************************/
  public void setOpenLoopControl() {
    setBrakeMode(true);
  }

  /****************************************************************************************************************************** 
  ** SET OPEN LOOP OUTPUT
  ******************************************************************************************************************************/
  public void setOpenLoopOutput(double zElevator) {
      ApplyDriveSignal(zElevator);
  }

 /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public Flywheel(WPI_TalonSRX master, WPI_TalonSRX follower) {
    mRightLeader = master;
    mLeftFollow = follower;

    mIsBrakeMode = false;
    setBrakeMode(true);

    mIsInverted = true;
    InvertOutput(false);

    // This is inverted alongside the joystick inputs in order to create working limit switches
    mRightLeader.setInverted(true);
    mLeftFollow.setInverted(true);

    // Start off in open loop control
    setOpenLoopControl();
  }

  /****************************************************************************************************************************** 
  ** OVERRIDE DEFAULT SUBSYSTEM COMMAND
  ******************************************************************************************************************************/
  // initdefaultcommand has been removed
  // @Override
  // public void initDefaultCommand() {
  //  setDefaultCommand(new FlywheelJoystick());
  // }

  public static Flywheel create() {
    WPI_TalonSRX mRightLeader = new WPI_TalonSRX(Constants.kRightFlyWheelLeader);
    WPI_TalonSRX mLeftFollow = new WPI_TalonSRX(Constants.kLeftFlyWheelFollower);

    return new Flywheel(mRightLeader, mLeftFollow);
  }

  public static void initDefaultSetup() {
  }

}

