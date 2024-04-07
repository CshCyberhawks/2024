package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command

class AutoIntake : Command() {

    var cannonIntake: AutoIntakeCannon = AutoIntakeCannon()
    var trunkIntake: AutoIntakeTrunk = AutoIntakeTrunk()

    override fun initialize() {
        cannonIntake = AutoIntakeCannon()
        trunkIntake = AutoIntakeTrunk()


        cannonIntake.schedule()
        trunkIntake.schedule()
    }

    override fun execute() {
    }

    override fun isFinished(): Boolean {
        return cannonIntake.isFinished && trunkIntake.isFinished
    }

    override fun end(interrupted: Boolean) {
        cannonIntake.cancel()
        trunkIntake.cancel()
    }
}
