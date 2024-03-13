package frc.robot.commands.automatic

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.DriveState
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.constants.DriveConstants

class AutoAimTwistAndShoot : Command() {

    val autoShoot: AutoShootCommand = AutoShootCommand()

    override fun initialize() {
//        RobotContainer.stateMachine.shooterState = ShooterState.Shooting
//        RobotContainer.stateMachine.driveState = DriveState.TranslationTeleop
//
//        if (RobotContainer.stateMachine.trunkState != TrunkState.Speaker && RobotContainer.stateMachine.trunkState != TrunkState.SpeakerFromStage) {
//            if (RobotContainer.stateMachine.currentRobotZone == GlobalZones.Stage) {
//                RobotContainer.stateMachine.trunkState = TrunkState.SpeakerFromStage
//            } else {
//                RobotContainer.stateMachine.trunkState = TrunkState.Speaker
//            }
//        }

        //Reset the controller
//        RobotContainer.swerveSystem.autoTwistController.reset(RobotContainer.swerveSystem.getSwervePose().rotation,
//                RobotContainer.swerveSystem.driveTrain.fieldVelocity)
        RobotContainer.swerveSystem.autoTwistController.reset(
            RobotContainer.swerveSystem.getSwervePose().rotation,
            RobotContainer.swerveSystem.driveTrain.currentRobotChassisSpeeds
        )
    }

    override fun execute() {
        //TODO: get the desired rotation (doesn't exist yet so using 0 as a placeholder)
        val desiredRot = 0.0

        val driveTwist =
            RobotContainer.swerveSystem.autoTwistController.calculateRotation(Rotation2d.fromDegrees(desiredRot))

        val driveTranslation = RobotContainer.swerveSystem.calculateJoyTranslation(
            RobotContainer.rightJoystick.x, RobotContainer.rightJoystick.y,
            RobotContainer.swerveSystem.calculateJoyThrottle(RobotContainer.leftJoystick.throttle),
            DriveConstants.TELEOP_DEADZONE_X,
            DriveConstants.TELEOP_DEADZONE_Y
        )
//        drive(driveTranslation, driveTwist.degrees, true)
        RobotContainer.swerveSystem.driveTrain.applyRequest {
            RobotContainer.swerveSystem.drive.withVelocityX(
                driveTranslation.x
            ).withVelocityY(driveTranslation.y).withRotationalRate(driveTwist.radians)
        }

        if (RobotContainer.swerveSystem.autoTwistController.isAtDesired() && !autoShoot.isScheduled) {
            autoShoot.schedule()
        }
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.driveState = DriveState.Teleop
    }
}
