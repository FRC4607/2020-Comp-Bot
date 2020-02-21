package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Aims at the target and shoots at least a given number of times.
 */
public class LimelightDrive extends CommandBase {

    private final Drivetrain mDrivetrain;
    private final Limelight mLimelight;

    // TODO: figure out PID values
    private final PIDController pidController = new PIDController( 0, 0, 0 );
    
    private final int ballsToShoot;
    // private int ballsShot = 0;
    private boolean noTarget = false;
    private boolean wasFull = false;
    private Timer endTimer = new Timer();

    public LimelightDrive( int ballsToShoot, Drivetrain drivetrain, Limelight limelight ) {

        // super( 100, limelight );

        this.ballsToShoot = ballsToShoot;
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        addRequirements( drivetrain, limelight );

        pidController.setTolerance(DRIVETRAIN.AIM_TOLERANCE);
    }

    @Override
    public void initialize() {
        noTarget = false;
        // ballsShot = 0;
        // wasFull = indexerSubsystem.isFull();
        endTimer.reset();
        pidController.reset();
        limelight.enable();
    }

    @Override
    public void execute() {
        var limelightWithTarget = getTargetAcquired();
        if (limelightWithTarget != null) {
            shooterSubsystem.prepareToShoot(Units.metersToInches(limelightWithTarget.getDistanceToTarget()));
            aimShooter(limelightWithTarget);
            if (shooterSubsystem.isReadyToShoot() && pidController.atSetpoint()) {
                indexerSubsystem.shoot();
            } else {
                indexerSubsystem.prepareToShoot();
            }
        } else {
            noTarget = true;
            mDrivetrain.stop();
        }

        var isFull = indexerSubsystem.isFull();
        if ((wasFull && !isFull) && (++ballsShot >= ballsToShoot)) {
            endTimer.start();
        }
        wasFull = isFull;
    }

    private void aimShooter(ILimelightSubsystem selectedLimelightSubsystem) {
        double targetX = selectedLimelightSubsystem.getTargetX(); //getFilteredX()
        double rotationSpeed = -pidController.calculate(targetX / 5);
        driveTrainSubsystem.arcadeDrive(0.0, rotationSpeed, false);
    }

    @Override
    public boolean isFinished() {
        return noTarget; // || (ballsShot >= ballsToShoot && endTimer.hasPeriodPassed(ShooterConstants.SHOOT_TIME));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        indexerSubsystem.stopIndexer();
        driveTrainSubsystem.stop();
    }
}

