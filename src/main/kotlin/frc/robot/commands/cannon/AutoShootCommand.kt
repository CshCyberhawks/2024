package frc.robot.commands.cannon

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class AutoShootCommand : Command() {

    var shooterReadyTime = -1.0
    override fun initialize() {
        RobotContainer.cannonSystem.shoot()
        shooterReadyTime = -1.0
    }

    override fun execute() {
        if (RobotContainer.stateMachine.shooterReady && shooterReadyTime < 0.0) {
            shooterReadyTime = Timer.getFPGATimestamp()
        }

        if (Timer.getFPGATimestamp() - shooterReadyTime > .15 && shooterReadyTime >= 0.0) {
            RobotContainer.cannonSystem.feed()
        }

        SmartDashboard.putBoolean("shooter ready", RobotContainer.stateMachine.shooterReady)
    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.cannonSystem.killIntake()
    }
}
