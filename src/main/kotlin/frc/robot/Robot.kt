package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

class Robot : LoggedRobot() {
    override fun robotInit() {
//        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
//        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE)
//        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA)
//        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE)
//        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH)
//
//        when (BuildConstants.DIRTY) {
//            0 -> Logger.recordMetadata("GitDirty", "All changes committed")
//            1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes")
//            else -> Logger.recordMetadata("GitDirty", "Unknown")
//        }

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
//        RobotContainer
//        Logger.start()
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


    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
//        RobotContainer.swerveSystem.swerveDrive.lockPose()
    }

    override fun disabledPeriodic() {}

    override fun autonomousInit() {
        RobotContainer.autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {
//        RobotContainer.autonomousCommand.cancel()
//        RobotContainer.teleopSwerveCommand.schedule()
//        RobotContainer.teleopElevateCommand.schedule()
        RobotContainer.teleopShootCommand.schedule()

    }

    override fun teleopPeriodic() {
        SmartDashboard.putNumber("JoyX", RobotContainer.rightJoystick.x)
        SmartDashboard.putNumber("JoyY", RobotContainer.rightJoystick.y)
        SmartDashboard.putNumber("JoyTwist", RobotContainer.rightJoystick.twist)
        val desiredState = SwerveModuleState(0.0, Rotation2d(0.0, 0.0))
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {
//        RobotContainer.swerveSystem.drive(Translation2d(0.25, 0.0), 0.0, true)
//        val calibrator = ShooterCalibrator("/u/shooter_calibrator/test1.csv");
//        calibrator.writeOut(shots)
//        val readShots = calibrator.readCsv();
//        readShots.forEach {
//            println(it.toCSV())
//        }
    }
}
