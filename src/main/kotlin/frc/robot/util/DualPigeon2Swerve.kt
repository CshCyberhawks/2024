package frc.robot.util

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.configs.Pigeon2Configurator
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Quaternion
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import swervelib.imu.SwerveIMU
import java.util.*

class DualPigeon2Swerve(canidNormal: Int, canidReverse: Int, canbusName: String, private var offset: Rotation3d) :
    SwerveIMU() {
    val normalPigeon: Pigeon2 = Pigeon2(canidNormal, canbusName)
    val reversePigeon: Pigeon2 = Pigeon2(canidReverse, canbusName)

    override fun factoryDefault() {
        val cfgN: Pigeon2Configurator = normalPigeon.configurator
        val cfgR: Pigeon2Configurator = reversePigeon.configurator

        val config = Pigeon2Configuration()

        cfgN.apply(config.Pigeon2Features.withEnableCompass(false))
        cfgR.apply(config.Pigeon2Features.withEnableCompass(false))
    }

    override fun clearStickyFaults() {
        normalPigeon.clearStickyFaults()
        reversePigeon.clearStickyFaults()
    }

    override fun setOffset(offset: Rotation3d) {
        this.offset = offset;
    }

    private fun getSingleRotation(imu: Pigeon2): Rotation3d {
        val w: StatusSignal<Double> = imu.quatW
        val x: StatusSignal<Double> = imu.quatX
        val y: StatusSignal<Double> = imu.quatY
        val z: StatusSignal<Double> = imu.quatZ
        return Rotation3d(
            Quaternion(
                w.refresh().value,
                x.refresh().value,
                y.refresh().value,
                z.refresh().value,
            )
        )
    }

    override fun getRawRotation3d(): Rotation3d {
        return getSingleRotation(normalPigeon).minus(getSingleRotation(reversePigeon)).div(2.0)
    }

    override fun getRotation3d(): Rotation3d {
        return getRawRotation3d().minus(offset)
    }

    private fun getSingleAccel(imu: Pigeon2): Optional<Translation3d> {
        val xAcc: StatusSignal<Double> = imu.accelerationX
        val yAcc: StatusSignal<Double> = imu.accelerationY
        val zAcc: StatusSignal<Double> = imu.accelerationZ

        return Optional.of(
            Translation3d(
                xAcc.refresh().value,
                yAcc.refresh().value,
                zAcc.refresh().value
            ).times(9.81 / 16384.0)
        )
    }

    override fun getAccel(): Optional<Translation3d> {
        return Optional.of(((getSingleAccel(normalPigeon).get()).minus(getSingleAccel(reversePigeon).get())).div(2.0))
    }


    override fun getIMU(): Any {
        return normalPigeon
    }
}