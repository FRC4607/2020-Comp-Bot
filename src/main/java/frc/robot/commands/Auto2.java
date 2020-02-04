package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Auto2 extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Drivetrain mDrivetrain;
    private boolean mIsFinished;

    @Override
    public void initialize () {}

    @Override
    public void execute() {

    }

    @Override
    public void end ( boolean interrupted ) {}

    @Override
    public boolean isFinished () {
      return mIsFinished;
    }

    public Auto2 ( Drivetrain drivetrain ) {
        mDrivetrain = drivetrain;
        mIsFinished = false;
        addRequirements(mDrivetrain);
        this.setName( "Auto2" );
    }

}
