package frc.robot.subsystems.swerve

import MiscCalculations
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.GlobalZones
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants
import frc.robot.constants.TunerConstants
import frc.robot.util.AutoTwistController

class SwerveSystem() : SubsystemBase() {
    val driveTrain: CommandSwerveDrivetrain = TunerConstants.DriveTrain

//    var inputRotation: Double = 0.0

    private val xPID: PIDController = PIDController(.1, 0.0, 0.01)
    private val yPID: PIDController = PIDController(.1, 0.0, 0.01)

    private val PIDDeadzone = .005;

    public val drive: SwerveRequest.FieldCentric = SwerveRequest.FieldCentric()
        .withDeadband(DriveConstants.MAX_SPEED * 0.1)
        .withRotationalDeadband(DriveConstants.MAX_ANGLE_SPEED * 0.1) // Add a 10% deadband
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    // driving in open loop
    public val brake: SwerveRequest.SwerveDriveBrake = SwerveRequest.SwerveDriveBrake();
    public val forwardStraight: SwerveRequest.RobotCentric =
        SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    public val point: SwerveRequest.PointWheelsAt = SwerveRequest.PointWheelsAt();

    /* Path follower */
//    public val runAuto: Command = driveTrain.getAutoPath("Tests");

    public val logger: Telemetry = Telemetry(DriveConstants.MAX_SPEED);


    val autoTwistController: AutoTwistController = AutoTwistController()

    //Takes in joystick inputs
    fun calculateJoyTranslation(
        rightX: Double,
        rightY: Double,
        throttle: Double,
        deadzoneX: Double,
        deadzoneY: Double
    ): Translation2d {
        return Translation2d(
            -MiscCalculations.calculateDeadzone(rightY, deadzoneX) * DriveConstants.MAX_SPEED * throttle,
            -MiscCalculations.calculateDeadzone(rightX, deadzoneY) * DriveConstants.MAX_SPEED * throttle
        )
    }

    fun calculateJoyThrottle(joyThrottle: Double): Double {
        //Milan: trust me bro this'll work totally definitely please don't question it
        return ((joyThrottle * -1) + 1) / 2
    }

    fun getSwervePose(): Pose2d {
        return driveTrain.state.Pose
    }

    fun zeroGyro() {
        driveTrain.seedFieldRelative()
    }

    //Updates the swerve drive position zone in the state machine
    fun updateGlobalZone() {
        val pos = getSwervePose().translation

        if (MiscCalculations.translation2dWithinRange(pos, RobotContainer.stateMachine.currentRobotZone.range)) {
            return
        }

        val currentRange = MiscCalculations.findMatchingTranslation2dRange(
            pos,
            arrayOf(GlobalZones.Wing.range, GlobalZones.NO.range, GlobalZones.Stage.range)
        )

        //Not in any zone
        if (currentRange.first.x == -1.0 && currentRange.second.y == -1.0) {
            println("Not in a valid zone; ERROR")
            return
        }

        val currentZone = when (currentRange) {
            GlobalZones.Stage.range -> GlobalZones.Stage
            GlobalZones.Wing.range -> GlobalZones.Wing
            GlobalZones.NO.range -> GlobalZones.NO
            else -> null
        }

        if (currentZone != null) {
            RobotContainer.stateMachine.prevRobotZone = RobotContainer.stateMachine.currentRobotZone
            RobotContainer.stateMachine.currentRobotZone = currentZone
        } else {
            println("Not in a valid zone; current zone returned was not default; ERROR")
        }
    }

    override fun periodic() {
//        updateGlobalZone()
        //TODO: update global (specific) positions in the state machine
    }
}