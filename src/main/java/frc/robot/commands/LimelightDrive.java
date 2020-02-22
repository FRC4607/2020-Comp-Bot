// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.util.Units;
// import frc.robot.Constants.DRIVETRAIN;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Limelight;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// /**
//  * Aims at the target and shoots at least a given number of times.
//  */
// public class LimelightDrive extends CommandBase {

//     private final Drivetrain mDrivetrain;
//     private final Limelight mLimelight;

//     // TODO: figure out PID values
//     private final PIDController pidController = new PIDController( 0, 0, 0 );
    
//    // private final int ballsToShoot;
//     // private int ballsShot = 0;
//     private boolean noTarget = false;
//     private boolean wasFull = false;
//     private Timer endTimer = new Timer();

//     public LimelightDrive( /*int ballsToShoot,*/ Drivetrain drivetrain, Limelight limelight ) {

//         // super( 100, limelight );

//         //this.ballsToShoot = ballsToShoot;
//         this.mDrivetrain = drivetrain;
//         this.mLimelight = limelight;

//         addRequirements( drivetrain, limelight );

//         pidController.setTolerance(DRIVETRAIN.AIM_TOLERANCE);
//     }

//     @Override
//     public void initialize() {
//         noTarget = false;
//         // ballsShot = 0;
//         // wasFull = indexerSubsystem.isFull();
//         endTimer.reset();
//         pidController.reset();
//         mLimelight.enable();
//     }

//     @Override
//     public void execute() {
//        // var limelightWithTarget = getTargetAcquired();
//         if ( limelightWithTarget != null ) {
//            /// aimRobot( limelightWithTarget );
//             if (mDrivetrain.isReadyToTurn() && pidController.atSetpoint()) {
//                 mDrivetrain.Turn();
//         } else {
//             noTarget = true;
//             mDrivetrain.Stop();
//         }
//     }

//     private void aimRobot( Limelight selectedLimelightSubsystem ) {
//         double targetX = selectedLimelightSubsystem.getTargetX(); //getFilteredX()
//         double rotationSpeed = -pidController.calculate(targetX / 5);
//         mDrivetrain.mDifferentialDrive.arcadeDrive(0.0, rotationSpeed, false);
//     }

//     @Override
//     public boolean isFinished() {
//         return noTarget; // || (ballsShot >= ballsToShoot && endTimer.hasPeriodPassed(ShooterConstants.SHOOT_TIME));
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mDrivetrain.Stop();
//     }
// }

