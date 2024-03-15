package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.automatic.AutoClimbCommand
import frc.robot.commands.trunk.CalibrateTrunk
import frc.robot.constants.CannonConstants
import frc.robot.util.Telemetry
import frc.robot.constants.TargetingConstants
import frc.robot.constants.TrunkConstants
import frc.robot.util.Math
import org.littletonrobotics.junction.LoggedRobot


class Robot : LoggedRobot() {
    private lateinit var m_autonomousCommand: Command

    private var lastRobotAction: RobotAction = RobotContainer.stateMachine.robotAction
    private var lastShootPosition: ShootPosition = RobotContainer.stateMachine.shootPosition

    private val autoClimbCommand: AutoClimbCommand = AutoClimbCommand()
    override fun robotInit() {
        SmartDashboard.putBoolean("arm motors free?", false)
        SmartDashboard.putNumber("varying shooter fudging constant", TargetingConstants.stupidConstant)
        SmartDashboard.putNumber("shooter endpoint x", TargetingConstants.endpointX)
        SmartDashboard.putNumber("shooter endpoint z", TargetingConstants.endpointZ)
        SmartDashboard.putNumber("shooter height", TargetingConstants.shooterZ)
        SmartDashboard.putNumber("shooter velocity transfer multiplier", TargetingConstants.velocityMultiplier)
        SmartDashboard.putNumber("constant shooter fudging constant", TargetingConstants.constantStupidConstant)
        SmartDashboard.putNumber("Current Target Angle", 0.0)
        RobotContainer.swerveSystem.driveTrain.getDaqThread().setThreadPriority(99);

        SmartDashboard.putNumber("shooter angle", 58.5)

        SmartDashboard.putBoolean("Shooter Ready & Aimed", false)

        SmartDashboard.putNumber("Amp Speed", CannonConstants.INNER_AMP_PERCENT)
        SmartDashboard.putNumber("Amp Angle", TrunkConstants.AMP_ANGLE)
        SmartDashboard.putNumber("Amp Position", TrunkConstants.AMP_POSITION)


        RobotContainer
    }

    override fun robotPeriodic() {
        RobotContainer.swerveSystem.logger.telemeterize(RobotContainer.swerveSystem.driveTrain.state)

        SmartDashboard.putBoolean("Is trunk ready?", RobotContainer.stateMachine.trunkReady)

        TargetingConstants.stupidConstant = SmartDashboard.getNumber("shooter fudging constant", TargetingConstants.stupidConstant)
        TargetingConstants.endpointX = SmartDashboard.getNumber("shooter endpoint x", TargetingConstants.endpointX)
        TargetingConstants.endpointZ = SmartDashboard.getNumber("shooter endpoint z", TargetingConstants.endpointZ)
        TargetingConstants.shooterZ = SmartDashboard.getNumber("shooter height", TargetingConstants.shooterZ)
        TargetingConstants.constantStupidConstant = SmartDashboard.getNumber("constant shooter fudging constant", TargetingConstants.constantStupidConstant)
        TargetingConstants.velocityMultiplier = SmartDashboard.getNumber("shooter velocity transfer multiplier", TargetingConstants.velocityMultiplier)

        CommandScheduler.getInstance().run()
        RobotContainer.stateMachine.logStates()


        val armMotorsFree = SmartDashboard.getBoolean("arm motors free?", false)

        if (armMotorsFree && !DriverStation.isTeleopEnabled() && !DriverStation.isTestEnabled() && !DriverStation.isAutonomousEnabled()) {
            RobotContainer.trunkSystem.freeMotors()
        } else if (!armMotorsFree && !DriverStation.isTeleopEnabled() && !DriverStation.isTestEnabled() && !DriverStation.isAutonomousEnabled()) {
            RobotContainer.trunkSystem.brakeMotors()
        }

        if (!DriverStation.isDisabled()) {
//            RobotContainer.visionSystem.updateOdometry(1, true)
            RobotContainer.visionSystem.updateOdometry(1, false)

        } else {
//            RobotContainer.visionSystem.updateOdometry(1, false)
            RobotContainer.visionSystem.updateOdometryFromDisabled()
        }


        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity()
        SmartDashboard.putNumber("Mathed shooter angle", shotSetup.shooterAngle)
        SmartDashboard.putNumber("Mathed robot angle", shotSetup.robotAngle)
        SmartDashboard.putNumber("Current robot angle", RobotContainer.swerveSystem.getSwervePose().rotation.degrees)

        Telemetry.putBoolean("shooter ready", RobotContainer.cannonSystem.shooterReady(), RobotContainer.telemetry.cannonTelemetry)
        Telemetry.putString("note state", RobotContainer.stateMachine.noteState.name, RobotContainer.telemetry.cannonTelemetry)
        Telemetry.putString("intake state", RobotContainer.stateMachine.intakeState.name, RobotContainer.telemetry.cannonTelemetry)


        SmartDashboard.putNumber("Robot X", RobotContainer.swerveSystem.getSwervePose().x)
        SmartDashboard.putNumber("Robot Y", RobotContainer.swerveSystem.getSwervePose().y)
        SmartDashboard.putNumber("Robot Angle", RobotContainer.swerveSystem.getSwervePose().rotation.degrees)

//        val changedPosition = Field2d.toWPILIBFieldPosition(FieldPosition(fieldPosition.x, fieldPosition.y, gyro.getYaw()))
//        field2d.get().setRobotPose(changedPosition.x, changedPosition.y, Rotation2d(changedPosition.angleRadians))

        SmartDashboard.putString("Current Trunk Command", RobotContainer.stateMachine.currentTrunkCommand.name)
        SmartDashboard.putNumber("Trunk Target Rotation", RobotContainer.trunkSystem.trunkDesiredRotation)
        SmartDashboard.putNumber("Trunk Rotation", RobotContainer.trunkSystem.getRotation())
        SmartDashboard.putNumber("Trunk Position", RobotContainer.trunkSystem.getPosition())

        SmartDashboard.putString("Cannon State", RobotContainer.stateMachine.intakeState.name)

        SmartDashboard.putNumber("Falcon Raw Rotation", RobotContainer.trunkSystem.io.getFalconRawRotation())
        SmartDashboard.putNumber("TB Raw Rotation", RobotContainer.trunkSystem.io.getThroughBoreRawRotation())
        SmartDashboard.putNumber("TB Rotation", Math.wrapAroundAngles((-RobotContainer.trunkSystem.io.getThroughBoreRawRotation() * 360.0) - TrunkConstants.rotationOffset))
        SmartDashboard.putNumber("Falcon Rotation", Math.wrapAroundAngles(RobotContainer.trunkSystem.io.getFalconRawRotation() * 360.0 - TrunkConstants.rotationOffset))

        CannonConstants.INNER_AMP_PERCENT = SmartDashboard.getNumber("Amp Speed", CannonConstants.INNER_AMP_PERCENT)
        TrunkConstants.AMP_ANGLE = SmartDashboard.getNumber("Amp Angle", TrunkConstants.AMP_ANGLE)
        TrunkConstants.AMP_POSITION = SmartDashboard.getNumber("Amp Position", TrunkConstants.AMP_POSITION)
        TrunkPose.AMP.angle = TrunkConstants.AMP_ANGLE
        TrunkPose.AMP.position = TrunkConstants.AMP_POSITION
        IntakeState.AmpSpitting.innerPercent = CannonConstants.INNER_AMP_PERCENT
        IntakeState.AmpSpitting.outerPercent = CannonConstants.INNER_AMP_PERCENT
    }


    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
//        RobotContainer.autonomousCommand.schedule()
//        RobotContainer.swerveSystem.zeroGyro()
//        DriveBackAuto().schedule()
        RobotContainer.swerveSystem.driveTrain.getAutoPath("Amp Side 4 Piece").schedule()
    }

    override fun autonomousPeriodic() {}

    override fun autonomousExit() {}

    override fun teleopInit() {
        RobotContainer.stateMachine.currentTrunkCommand = CalibrateTrunk()
        RobotContainer.stateMachine.currentTrunkCommand.schedule()

        RobotContainer.cannonSystem.killShooter()
        RobotContainer.cannonSystem.killIntake()


        RobotContainer.autonomousCommand.cancel()
        RobotContainer.teleopSwerveCommand.schedule()
//        RobotContainer.trunkSystem.calibrate()

//        RobotContainer.trunkSystem.STOP()


        SmartDashboard.putBoolean("Schedule Climb Command?", false)
        SmartDashboard.putBoolean("Pulldown Climb?", false)
    }

    override fun teleopPeriodic() {
        RobotContainer.stateMachine.TeleopAutomaticStateManagement()


        val scheduleClimbBool = SmartDashboard.getBoolean("Schedule Climb Command?", false)
        if (scheduleClimbBool && autoClimbCommand.isScheduled() == false) {
            autoClimbCommand.schedule()
        } else if (autoClimbCommand.isScheduled == true && scheduleClimbBool == false) {
            autoClimbCommand.cancel()
        }


        //        SmartDashboard.putNumber("JoyX", RobotContainer.rightJoystick.x)
        //        SmartDashboard.putNumber("JoyY", RobotContainer.rightJoystick.y)
        //        SmartDashboard.putNumber("JoyTwist", RobotContainer.rightJoystick.twist)
        //        val desiredState = SwerveModuleState(0.0, Rotation2d(0.0, 0.0))
        //        if(RobotContainer.xboxController.b().asBoolean)
        //            RobotContainer.trunkSystem.goManual()
        //        if(RobotContainer.xboxController.x().asBoolean)
        //            RobotContainer.trunkSystem.STOP()
        //        if(RobotContainer.xboxController.a().asBoolean)
        //            RobotContainer.trunkSystem.calibrate()
        //        RobotContainer.trunkSystem.elevate(-RobotContainer.xboxController.leftY)
        //        RobotContainer.trunkSystem.rotate(-RobotContainer.xboxController.rightY*.3)

    }

    override fun teleopExit() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {

//        RobotContainer.targetingSystem.test()
    }

    override fun testExit() {}

    override fun simulationPeriodic() {}
}
