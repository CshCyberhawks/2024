package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.commands.trunk.GoToPoseTrunk
import frc.robot.commands.trunk.HoldPositionGoToAngleTrunk
import frc.robot.commands.trunk.StowTrunkCommand
import frc.robot.constants.DriveConstants
import frc.robot.constants.TrunkConstants
import frc.robot.util.ProfiledPID

class FerryShot: Command() {
    var autoShoot: AutoShootCommand = AutoShootCommand()

    var trunkCommand: HoldPositionGoToAngleTrunk = HoldPositionGoToAngleTrunk(TrunkPose.SPEAKER)

    val twistPIDController = ProfiledPID(10.0, 0.0, 0.1, TrapezoidProfile.Constraints(100.0, 50.0))

    private var lastRobotAngle = 0.0

    val shooterAngle: Double = 80.0
    val ferryRobotAngle: Double = 180.0

    override fun initialize() {
        trunkCommand = HoldPositionGoToAngleTrunk(TrunkPose.SPEAKER)
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting
        autoShoot = AutoShootCommand()

        RobotContainer.stateMachine.driveState = DriveState.TranslationTeleop
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting
        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseTrunk(TrunkPose.SPEAKER).andThen(trunkCommand);
        RobotContainer.actuallyDoShoot = false

        twistPIDController.enableContinuousInput(0.0, 360.0);
        twistPIDController.goal = lastRobotAngle
    }

    override fun execute() {
        trunkCommand.desiredAngle = shooterAngle

        if (lastRobotAngle != ferryRobotAngle) {
            twistPIDController.goal = ferryRobotAngle
        }

        //Handle the twisting component
        val driveTwist = twistPIDController.calculate(
            RobotContainer.swerveSystem.getSwervePose().rotation.degrees,
        )

        val driveTranslation = RobotContainer.swerveSystem.calculateJoyTranslation(
            RobotContainer.rightJoystick.x, RobotContainer.rightJoystick.y,
            RobotContainer.swerveSystem.calculateJoyThrottle(RobotContainer.leftJoystick.throttle),
            DriveConstants.TELEOP_DEADZONE_X,
            DriveConstants.TELEOP_DEADZONE_Y
        )

        RobotContainer.swerveSystem.applyDriveRequest(
            driveTranslation.x,
            driveTranslation.y,
            Math.toRadians(driveTwist)
        ).execute()

        if (RobotContainer.actuallyDoShoot && !autoShoot.isScheduled) {
            autoShoot.schedule()
        }

        lastRobotAngle = ferryRobotAngle
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.currentTrunkCommand = StowTrunkCommand()
        RobotContainer.actuallyDoShoot = false
    }
}