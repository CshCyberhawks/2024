package frc.robot.subsystems.swerve

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit

class SwerveTelemetry
/**
 * Construct a telemetry object, with the specified max speed of the robot
 *
 * @param maxSpeed Maximum speed in meters per second
 */(private val MaxSpeed: Double) {
    /* What to publish over networktables for telemetry */
    private val inst: NetworkTableInstance = NetworkTableInstance.getDefault()

    /* Robot pose for field positioning */
    private val table: NetworkTable = inst.getTable("Pose")
    private val fieldPub: DoubleArrayPublisher = table.getDoubleArrayTopic("robotPose").publish()
    private val fieldTypePub: StringPublisher = table.getStringTopic(".type").publish()

    /* Robot speeds for general checking */
    private val driveStats: NetworkTable = inst.getTable("Drive")
    private val velocityX: DoublePublisher = driveStats.getDoubleTopic("Velocity X").publish()
    private val velocityY: DoublePublisher = driveStats.getDoubleTopic("Velocity Y").publish()
    private val speed: DoublePublisher = driveStats.getDoubleTopic("Speed").publish()
    private val odomPeriod: DoublePublisher = driveStats.getDoubleTopic("Odometry Period").publish()

    /* Keep a reference of the last pose to calculate the speeds */
    private var m_lastPose = Pose2d()
    private var lastTime = Utils.getCurrentTimeSeconds()

    /* Mechanisms to represent the swerve module states */
    private val m_moduleMechanisms = arrayOf(
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
    )

    /* A direction and length changing ligament for speed representation */
    private val m_moduleSpeeds = arrayOf(
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(MechanismLigament2d("Speed", 0.5, 0.0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(MechanismLigament2d("Speed", 0.5, 0.0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(MechanismLigament2d("Speed", 0.5, 0.0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(MechanismLigament2d("Speed", 0.5, 0.0)),
    )

    /* A direction changing and length constant ligament for module direction */
    private val m_moduleDirections = arrayOf(
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
    )

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    fun telemeterize(state: SwerveDriveState) {
        /* Telemeterize the pose */
        val pose = state.Pose ?: Pose2d()
        fieldTypePub.set("Field2d")
        fieldPub.set(
            doubleArrayOf(
                pose.x,
                pose.y,
                pose.rotation.degrees
            )
        )

//        /* Telemeterize the robot's general speeds */
//        val currentTime = Utils.getCurrentTimeSeconds()
//        val diffTime = currentTime - lastTime
//        lastTime = currentTime
//        val distanceDiff = pose.minus(m_lastPose).translation
//        m_lastPose = pose
//
//        val velocities = distanceDiff.div(diffTime)
//
//        speed.set(velocities.norm)
//        velocityX.set(velocities.x)
//        velocityY.set(velocities.y)
//        odomPeriod.set(state.OdometryPeriod)
//
//        /* Telemeterize the module's states */
//        for (i in 0..3) {
//            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle)
//            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle)
//            m_moduleSpeeds[i].length = state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed)
//
//            SmartDashboard.putData("Module $i", m_moduleMechanisms[i])
//        }
    }
}
