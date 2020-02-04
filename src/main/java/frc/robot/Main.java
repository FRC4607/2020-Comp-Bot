package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import org.slf4j.LoggerFactory;
import ch.qos.logback.classic.LoggerContext;
import ch.qos.logback.classic.joran.JoranConfigurator;
import ch.qos.logback.core.joran.spi.JoranException;
import ch.qos.logback.core.util.StatusPrinter;

public final class Main {
    private Main() {
    }

    public static void main (String... args) {

        final File mLogBackFile = new File( Filesystem.getDeployDirectory(), "logback.xml" );
        LoggerContext context = (LoggerContext) LoggerFactory.getILoggerFactory();
        try {
          JoranConfigurator configurator = new JoranConfigurator();
          configurator.setContext( context );
          context.reset();
          configurator.doConfigure( mLogBackFile );
        } catch ( JoranException je ) {
        }
        StatusPrinter.printInCaseOfErrorsOrWarnings( context );

        RobotBase.startRobot( Robot::new );
    }
}