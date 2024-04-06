package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.cannon.HalfSpitCannon
import frc.robot.commands.cannon.IntakeCannon
import frc.robot.commands.trunk.*

class AutoIntakeTrunk : Command() {

    //    var intakePrepCommand = GoToPoseTrunk(TrunkPose.INTAKE_PREP)
//    val coastOutCommand = CoastAngleMovePosition(TrunkPose.INTAKE).andThen(CoastAngleHoldPosition(TrunkPose.INTAKE))
    var intakeOutCommand = GoToPoseAndCoast(TrunkPose.INTAKE, 0.38)
    val stowCommand = CoastAngleMovePosition(TrunkPose.STOW).andThen(StowTrunkCommand())
    // val stowCommand = StowTrunkCommand()

    var stowing = false

    override fun initialize() {
        RobotContainer.cannonSystem.killShooter()
        intakeOutCommand = GoToPoseAndCoast(TrunkPose.INTAKE, 0.3)

//        intakePrepCommand = GoToPoseTrunk(TrunkPose.INTAKE_PREP)

        RobotContainer.stateMachine.currentTrunkCommand = intakeOutCommand
        stowing = false
    }


    override fun execute() {
        /*if (intakePrepCommand.isFinished && RobotContainer.stateMachine.currentTrunkCommand == intakePrepCommand) {
            RobotContainer.stateMachine.currentTrunkCommand = coastOutCommand
        } else*/ if (RobotContainer.stateMachine.currentTrunkCommand == intakeOutCommand && (RobotContainer.stateMachine.noteState == NoteState.Stored)) {
            RobotContainer.stateMachine.currentTrunkCommand = stowCommand
            stowing = true
        }
    }

    override fun isFinished(): Boolean {
        if (stowing && RobotContainer.trunkSystem.isAtPose) {
            return true
        }
//        println("trunk intake is finished is at pose: ${RobotContainer.trunkSystem.isAtPose}, current trunk command: ${RobotContainer.stateMachine.currentTrunkCommand.name}")
        return false
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            stowCommand.cancel()
//            coastOutCommand.cancel()
//            intakePrepCommand.cancel()
            intakeOutCommand.cancel()

            stowCommand.schedule()
        }
    }
}
