package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.TransferWheel;
import frc.robot.commands.FlywheelSpin;

public class Auto1 extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Drivetrain mDrivetrain;
    private final Flywheel mFlywheel;
    private final Hopper mHopper;
    private final Indexer mIndexer;
    private final TransferWheel mTransferWheel;
    private boolean mIsFinished = false;

    @Override
    public void initialize () {
    }

    @Override
    public void execute() {
      //Start Flywheel
      //  mFlywheel.setOpenLoop( -0.6 );

      //  //Dont tell Eric, but wait for 3 seconds
      //  long start = System.currentTimeMillis();

      //  while( System.currentTimeMillis() <= start + 3000 ) {}

      //  //Feed Balls into Shooter
      //  mHopper.Spin();
      //  mIndexer.Spin();
      //  mTransferWheel.Spin();

      //  //Dont tell Eric again, but wait until balls are shot and stop everything
      //  start = System.currentTimeMillis();

      //  while( System.currentTimeMillis() <= start + 7000 ) {}

      //  mFlywheel.Stop();
      //  mHopper.Stop();
      //  mIndexer.Stop();
      //  mTransferWheel.Stop();

      //  //Drive off Line
      //  mDrivetrain.mDifferentialDrive.arcadeDrive(-.50, -.50);
      //  while(System.currentTimeMillis() <= start + 800) {}
      //  mDrivetrain.mDifferentialDrive.arcadeDrive(0, 0);

       // :D
       mIsFinished = true;
    }

    @Override
    public void end ( boolean interrupted ) {}

    @Override
    public boolean isFinished () {
      return mIsFinished;
    }

    public Auto1 ( Drivetrain drivetrain, Flywheel flywheel, Hopper hopper, Indexer indexer, TransferWheel transferWheel ) {
        mDrivetrain = drivetrain;
        mFlywheel = flywheel;
        mHopper = hopper;
        mIndexer = indexer;
        mTransferWheel = transferWheel;
        mIsFinished = false;
        addRequirements( mDrivetrain );
        addRequirements( mFlywheel );
        addRequirements( mHopper );
        addRequirements( mIndexer );
        addRequirements( mTransferWheel );
        this.setName( "Auto1" );
    }

}

