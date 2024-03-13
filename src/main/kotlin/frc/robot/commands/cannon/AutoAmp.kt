package frc.robot.commands.cannon

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPosition

class AutoAmp : Command() {

    override fun initialize() {
//        RobotContainer.cannonSystem.ampSpit()
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.AMP
//        RobotContainer.trunkSystem.goToCustom()
    }

    override fun execute() {
//        if (RobotContainer.stateMachine.trunkReady) {
//            RobotContainer.cannonSystem.ampSpit()
//        }
//        if (RobotContainer.xboxController.start().asBoolean) {
//            RobotContainer.cannonSystem.ampSpit()
//        } else {
        RobotContainer.cannonSystem.killIntake()
//        }
    }

    override fun isFinished(): Boolean {
//        return RobotContainer.stateMachine.noteState == NoteState.Empty
//        return false
        return RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.AMP
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.STOW
//        println("amp end")
    }
}