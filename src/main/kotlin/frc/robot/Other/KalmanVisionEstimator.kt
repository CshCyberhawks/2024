import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem

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


    fun updateOdometry(ll1: String, ll2: String) {
        val pos1 = Limelight
    }

}
