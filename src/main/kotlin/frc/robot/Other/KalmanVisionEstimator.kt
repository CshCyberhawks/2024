import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.RobotContainer
import limelightlib.LimelightHelpers

object KalmanVisionEstimator {

    private val mat_A = MatBuilder(Nat.N2(), Nat.N2()).fill(0.0, 0.0, 0.0, 0.0)

    private val mat_B = MatBuilder(Nat.N2(), Nat.N2()).fill(1.0, 0.0, 0.0, 1.0)

    private val mat_C = MatBuilder(Nat.N2(), Nat.N2()).fill(1.0, 0.0, 0.0, 1.0)

    private val mat_D = MatBuilder(Nat.N2(), Nat.N2()).fill(0.0, 0.0, 0.0, 0.0)

    private val stateStdDevs =
            MatBuilder(Nat.N2(), Nat.N1())
                    .fill(
                            .1,
                            .1,
                    )

    private val measureStdDevs = MatBuilder(Nat.N2(), Nat.N1()).fill(.75, .75)

    private val linearSystem: LinearSystem<N2, N2, N2> = LinearSystem(mat_A, mat_B, mat_C, mat_D)
    private val kalmanFilter: KalmanFilter<N2, N2, N2> =
            KalmanFilter(Nat.N2(), Nat.N2(), linearSystem, stateStdDevs, measureStdDevs, .02)

    fun setInitialMeasure() {
        val odometry_pos =
                RobotContainer.swerveSystem.swerveDrive.swerveDrivePoseEstimator
                        .getEstimatedPosition()


        val measure = MatBuilder(Nat.N2(), Nat.N1()).fill(odometry_pos.x, odometry_pos.y)
        kalmanFilter.setXhat(measure)
    }


    fun updateOdometry(ll1: String, ll2: String, joyx: Double, joyy: Double) {
        val pos1 = LimelightHelpers.getBotPose(ll1)
        val pos2 = LimelightHelpers.getBotPose(ll2)

        SmartDashboard.putNumber("ll right x:", pos1[0]);
        SmartDashboard.putNumber("ll left x:", pos2[0]);

        //get TV == has target
        if (!LimelightHelpers.getTV(ll1) && !LimelightHelpers.getTV(ll2)) {

        }
        //ll1 doesn't have; ll2 does
        else if (!LimelightHelpers.getTV(ll1)) {

        }
        //ll2 doesn't have; ll1 does
        else if (!LimelightHelpers.getTV(ll2)) {

        }


        val odometry_pos =
                RobotContainer.swerveSystem.swerveDrive.swerveDrivePoseEstimator
                        .getEstimatedPosition()

        val wheel_vel = RobotContainer.swerveSystem.swerveDrive.robotVelocity

        //predict step
        val u = MatBuilder(Nat.N2(), Nat.N1()).fill(joyx, joyy)
        //TODO: put actual correct dt in
        

        //update/correct step
        val y = MatBuilder(Nat.N2(), Nat.N1()).fill(odometry_pos.x, odometry_pos.y)
        kalmanFilter.correct(u, y)


        //get the new state and reset the odometry
        val xhat = kalmanFilter.getXhat()
        val new_pos = Translation2d(xhat[0, 0], xhat[1, 0])

        RobotContainer.swerveSystem.setPos(new_pos)
    }
}
