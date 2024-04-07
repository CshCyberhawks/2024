package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.RobotContainer
import frc.robot.commands.cannon.HalfSpitCannon
import frc.robot.commands.cannon.IntakeCannon

class AutoIntakeCannon : Command() {

    val intakeCannonCommand = IntakeCannon()

    var intakeCommand = SequentialCommandGroup(
        IntakeCannon(),
        HalfSpitCannon(),
        WaitCommand(.05),
        intakeCannonCommand,
    )

    override fun initialize() {
        RobotContainer.cannonSystem.killShooter()
        intakeCommand.schedule()
    }

    private fun cancelCommand() {
        intakeCommand.cancel()
    }

    override fun execute() {
    }

    override fun isFinished(): Boolean {
        return intakeCannonCommand.isFinished
    }

    override fun end(interrupted: Boolean) {
        intakeCommand.cancel()
    }
}
