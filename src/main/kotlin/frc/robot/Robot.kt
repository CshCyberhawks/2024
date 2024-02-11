package frc.robot

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : LoggedRobot() {
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE)
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA)
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE)
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH)

        when (BuildConstants.DIRTY) {
            0 -> Logger.recordMetadata("GitDirty", "All changes committed")
            1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes")
            else -> Logger.recordMetadata("GitDirty", "Unknown")
        }

        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
                PowerDistribution(1, PowerDistribution.ModuleType.kRev)
            }
            Constants.Mode.SIM -> {
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(NT4Publisher())
            }
            Constants.Mode.REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false) // Run as fast as possible
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }

        Logger.start()

        RobotContainer
        /* User can change the configs if they want, or leave it empty for factory-default */
//        canCoder.getConfigurator().apply(toApply)
    }


    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */

    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removin    g finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
//        val left = "limelight-left"
//        val right = "limelight-right"
//
//        val leftPose = LimelightHelpers.getBotPose2d(left)
//        val rightPose = LimelightHelpers.getBotPose2d(right)
//        if (LimelightHelpers.getTV(left) && LimelightHelpers.getTV(right))
//            RobotContainer.swerveSystem.swerveDrive.addVisionMeasurement(
//                Pose2d(
//                    leftPose.translation.plus(rightPose.translation).div(2.0),
//                    leftPose.rotation.plus(rightPose.rotation).div(2.0)
//                ), Timer.getFPGATimestamp()
//            )
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
        RobotContainer.swerveSystem.swerveDrive.lockPose()
    }

    /**
     * This function is called periodically when disabled.
     */
    override fun disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your [RobotContainer] class.
     */
    override fun autonomousInit() {
        // Schedule the autonomous command (example)
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before scheduling it
        RobotContainer.autonomousCommand.schedule()
    }

    /**
     * This function is called periodically during autonomous.
     */
    override fun autonomousPeriodic() {}

    /**
     * This function is called once when teleop is enabled.
     */
    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before cancelling it
        RobotContainer.armSystem.resetPosition()

        RobotContainer.autonomousCommand.cancel()
        RobotContainer.teleopSwerveCommand.schedule()
        RobotContainer.teleopArmCommand.schedule()
        RobotContainer.teleopClawCommand.schedule()
    }

    /**
     * This function is called periodically during operator control.
     */
    override fun teleopPeriodic() {}

    /**
     * This function is called once when test mode is enabled.
     */
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /**
     * This function is called periodically during test mode.
     */
    override fun testPeriodic() {
        RobotContainer.swerveSystem.swerveDrive.modules[0].setAngle(0.0)
        RobotContainer.swerveSystem.swerveDrive.modules[1].setAngle(0.0)
        RobotContainer.swerveSystem.swerveDrive.modules[2].setAngle(0.0)
        RobotContainer.swerveSystem.swerveDrive.modules[3].setAngle(0.0)
    }
}
