package frc.robot.commands.automatic

import MiscCalculations
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.constants.DriveConstants

class DumbAutoAimTwistAndShoot : Command() {
    val autoShoot: AutoShootCommand = AutoShootCommand()

    val twistPIDController: PIDController = PIDController(0.1, 0.0, 0.0)

    override fun initialize() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting
        RobotContainer.stateMachine.driveState = DriveState.TranslationTeleop

        if (RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.SPEAKER && RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.SPEAKER_FROM_STAGE) {
            if (RobotContainer.stateMachine.currentRobotZone == GlobalZones.Stage) {
                RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER_FROM_STAGE
            } else {
                RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER
            }
        }

        twistPIDController.enableContinuousInput(0.0, 360.0);


    }

    override fun execute() {
        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity()

        //Handle the cannon aiming component
//        val shooterAngle = Math.clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)
//        SmartDashboard.putBoolean("shot is possible?", shooterAngle == shotSetup.shooterAngle)
//        RobotContainer.trunkSystem.setShootingAngle(shooterAngle)


        //Handle the twisting component
        val driveTwist = twistPIDController.calculate(
            RobotContainer.swerveSystem.getSwervePose().rotation.degrees,
            shotSetup.robotAngle
        )

        val driveTranslation = RobotContainer.swerveSystem.calculateJoyTranslation(
            RobotContainer.rightJoystick.x, RobotContainer.rightJoystick.y,
            RobotContainer.swerveSystem.calculateJoyThrottle(RobotContainer.leftJoystick.throttle),
            DriveConstants.TELEOP_DEADZONE_X,
            DriveConstants.TELEOP_DEADZONE_Y
        )

        RobotContainer.swerveSystem.driveTrain.applyRequest {
            RobotContainer.swerveSystem.drive.withVelocityX(driveTranslation.x).withVelocityY(driveTranslation.y)
                .withRotationalRate(driveTwist)
        }


        //Can we shoot?
        if (RobotContainer.stateMachine.trunkReady && MiscCalculations.appxEqual(
                twistPIDController.setpoint,
                shotSetup.robotAngle,
                1.0
            ) && !autoShoot.isScheduled
        ) {
            autoShoot.schedule()
        }
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.driveState = DriveState.Teleop
    }
}