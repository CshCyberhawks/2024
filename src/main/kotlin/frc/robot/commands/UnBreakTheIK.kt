package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class UnBreakTheIK: Command() {
    var stepOne = false
    var stepTwo = false
    var stepThree = false

    override fun initialize() {
        stepOne = false
        stepTwo = false
        stepThree = false
    }

    override fun execute() {
        if (!stepOne) {
            RobotContainer.trunkSystem.STOP()
            stepOne = true
        }
        else if (!stepTwo) {
            RobotContainer.stateMachine.targetTrunkPose = TrunkPose.STOW
            stepTwo = true
        }
        else if (!stepThree) {
            RobotContainer.trunkSystem.goToCustom()
            stepThree = true
        }
    }

    override fun isFinished(): Boolean {
        return stepOne && stepTwo && stepThree
    }

}