package frc.robot.commands.autonomous

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer

class DriveBackAuto : Command() {
    var startTime: Double = -1.0
    override fun initialize() {
        startTime = Timer.getFPGATimestamp()
    }

    override fun execute() {
        RobotContainer.swerveSystem.driveTrain.applyRequest {
            RobotContainer.swerveSystem.drive.withVelocityX(0.5)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        }.execute()
    }

    override fun isFinished(): Boolean {
        return (Timer.getFPGATimestamp() - startTime >= 5.0) && startTime >= 0.0
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.swerveSystem.driveTrain.applyRequest {
            RobotContainer.swerveSystem.drive.withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        }.execute()
    }
}