package frc.robot.commands.trunk

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.constants.TrunkConstants

class GoToPoseAndHoldTrunk(val desiredPose: TrunkPose) : Command() {
    val goToPose = GoToPoseTrunk(desiredPose)

    override fun initialize() {
        goToPose.schedule()
    }


    override fun execute() {}

    override fun isFinished(): Boolean {
        return goToPose.isFinished
    }

    override fun end(interrupted: Boolean) {
        if (interrupted == false) {
            RobotContainer.trunkSystem.isAtPose = true
        }
        RobotContainer.stateMachine.currentTrunkCommand = HoldPoseTrunk(desiredPose)
        goToPose.cancel()
    }
}