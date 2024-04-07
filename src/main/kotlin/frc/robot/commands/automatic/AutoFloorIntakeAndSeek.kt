package frc.robot.commands.automatic

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.DriveState
import frc.robot.LimelightHelpers
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.util.Timer
import kotlin.math.abs

class AutoFloorIntakeAndSeek : Command() {
    var initTime: Double = -1.0
    val firstTrackTimer = Timer()

    init {
        SmartDashboard.putNumber("Limelight X Offset", 0.0)
        SmartDashboard.putNumber("Limelight Rotation Speed", 0.0)
    }

    override fun initialize() {
//        autoIntake.schedule()
//        initTime = Timer.getFPGATimestamp()
        RobotContainer.stateMachine.driveState = DriveState.Auto
        SmartDashboard.putBoolean("Floor intake and seek scheduled", true)
        firstTrackTimer.reset()
    }

    override fun execute() {
        if (LimelightHelpers.getTV(RobotContainer.intakeLimelight) && !firstTrackTimer.isRunning) {
//            firstTrackTimer.start()
        }


        if (LimelightHelpers.getTV(RobotContainer.intakeLimelight)) {
            val xOffset = LimelightHelpers.getTX(RobotContainer.intakeLimelight) * 5.0
            val fudgedLLOffset = Math.toRadians(if (abs(xOffset) < 1.0) 0.0 else xOffset)
            SmartDashboard.putNumber("Limelight X Offset", xOffset)
            SmartDashboard.putNumber("Limelight Rotation Speed", fudgedLLOffset)
            //Milan - everything needs to be negated bc the front is STUPID "sHoOtTeR sHoUlD be FrONt"
            RobotContainer.swerveSystem.driveTrain.applyRequest {
                RobotContainer.swerveSystem.forwardStraight.withVelocityX(
                    -1.0
                ).withRotationalRate(-fudgedLLOffset)
            }.execute()
        }
    }

    override fun isFinished(): Boolean {
//
//        //Doesn't use the auto intake just in case there is some sort of an issue with its ending and starting with a note already in the thing...idk - Milan
//        SmartDashboard.putBoolean(
//            "Floor intake and seek finished",
//            RobotContainer.stateMachine.noteState == NoteState.Stored
//        )
        return RobotContainer.stateMachine.noteState == NoteState.Stored
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.driveState = DriveState.Teleop
//        if (!autoIntake.isScheduled) {
//            autoIntake.cancel()
//        }
    }
}
