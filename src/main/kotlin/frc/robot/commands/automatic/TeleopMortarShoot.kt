package frc.robot.commands.automatic

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoMortarCommand
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.HoldPositionGoToAngleTrunk
import frc.robot.constants.DriveConstants
import frc.robot.constants.TrunkConstants
import frc.robot.util.ControllerUtil

class TeleopMortarShoot() : Command() {

    val autoShoot = AutoMortarCommand()

    val trunkCommand: HoldPositionGoToAngleTrunk = HoldPositionGoToAngleTrunk(TrunkPose.SPEAKER)

    val twistPIDController =
        //        ProfiledPIDController(1.0, 0.0, 0.1, TrapezoidProfile.Constraints(540.0, 720.0))
        PIDController(8.0, 0.0, 0.15)

    init {
//        SmartDashboard.putData("Twist PID Controller", twistPIDController)
    }

    override fun initialize() {
        RobotContainer.stateMachine.driveState = DriveState.TranslationTeleop
        RobotContainer.stateMachine.shooterState = ShooterState.Mortaring
        RobotContainer.stateMachine.currentTrunkCommand = trunkCommand;
        RobotContainer.actuallyDoShoot = false

        //        twistPIDController.reset(RobotContainer.swerveSystem.getSwervePose().rotation.degrees)
        twistPIDController.enableContinuousInput(0.0, 360.0);
    }

    override fun execute() {
        val shotSetup = RobotContainer.targetingSystem.getMortarShot(
            RobotContainer.swerveSystem.getSwervePose().plus(
                Transform2d(Translation2d(0.0, -1.0), Rotation2d())
            )
        )

        //        println("Shot Angle: ${shotSetup.shooterAngle}")

        SmartDashboard.putNumber("Swerve Twist PID Velocity Error", twistPIDController.velocityError)
        SmartDashboard.putNumber("Robot Shot Angle", shotSetup.robotAngle)
        SmartDashboard.putNumber("Robot Shot Input Angle", RobotContainer.swerveSystem.getSwervePose().rotation.degrees)

        //Handle the cannon aiming component
        val shooterAngle = clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)
        //        SmartDashboard.putBoolean("shot is possible?", shooterAngle == shotSetup.shooterAngle)
        //        RobotContainer.trunkSystem.setShootingAngle(shooterAngle)
        trunkCommand.desiredAngle = shooterAngle

        //Handle the twisting component
        val driveTwistPIDOutRadians =
            twistPIDController.calculate(
                RobotContainer.swerveSystem.getSwervePose().rotation.radians,
                Math.toRadians(shotSetup.robotAngle)
            )

        SmartDashboard.putNumber("Drive Twist", driveTwistPIDOutRadians)

        val driveTranslation = ControllerUtil.calculateJoyTranslation(
            RobotContainer.rightJoystick.x, RobotContainer.rightJoystick.y,
            ControllerUtil.calculateJoyThrottle(RobotContainer.leftJoystick.throttle),
            DriveConstants.TELEOP_DEADZONE_X,
            DriveConstants.TELEOP_DEADZONE_Y
        )

        RobotContainer.swerveSystem.applyDriveRequest(
            driveTranslation.x,
            driveTranslation.y,
            driveTwistPIDOutRadians
        ).execute()

        //Can we shoot?
        //        if (RobotContainer.stateMachine.trunkReady && MiscCalculations.appxEqual(
        //                        twistPIDController.setpoint,
        //                        shotSetup.robotAngle,
        //                        1.0
        //                ) && !autoShoot.isScheduled
        //        ) {
        //            autoShoot.schedule()
        //        }
        if (RobotContainer.actuallyDoShoot && !autoShoot.isScheduled) {
            autoShoot.schedule()
        }
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)
        RobotContainer.actuallyDoShoot = false
        RobotContainer.stateMachine.driveState = DriveState.Teleop
        RobotContainer.cannonSystem.killShooter()
    }
}