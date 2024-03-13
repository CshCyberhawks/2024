package frc.robot.commands.automatic


import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.constants.TrunkConstants


class AutoAimAndShoot : Command() {
    val autoShoot: AutoShootCommand = AutoShootCommand()

    val waitForTwist: Boolean = true
    var shooterAngle = 0.0

    override fun initialize() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting

        RobotContainer.stateMachine.targetTrunkPose = TrunkPose.SPEAKER
        RobotContainer.trunkSystem.goToAim()
//        if (RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.SPEAKER && RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.SPEAKER_FROM_STAGE) {
//            if (RobotContainer.stateMachine.currentRobotZone == GlobalZones.Stage) {
//                RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER_FROM_STAGE
//            } else {
//                RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER
//            }
//        }

        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity()

        shooterAngle = clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)

    }

    override fun execute() {
        SmartDashboard.putNumber("shooter angle", shooterAngle)
        RobotContainer.trunkSystem.setShootingAngle(shooterAngle)

        if (RobotContainer.xboxController.leftTrigger().asBoolean && !autoShoot.isScheduled) {
            autoShoot.schedule()
        }
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        println("Shootyboi Done")
//        RobotContainer.stateMachine.shooterState = ShooterState.Stopped
//        RobotContainer.stateMachine.driveState = DriveState.Teleop
        RobotContainer.stateMachine.targetTrunkPose = TrunkPose.STOW
        RobotContainer.trunkSystem.goToCustom()

    }
}