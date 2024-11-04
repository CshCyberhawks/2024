package frc.robot.util

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel.MotorType

class EmptyPositionValue(val value: Double = 0.0)

class EmptyConfigurator {
    fun apply(config: TalonFXConfiguration) {}
}

class EmptyMotorController(id: Int, motorType: MotorType = MotorType.kBrushless) {
    var inverted = false
    var position = EmptyPositionValue()
    var velocity = EmptyPositionValue()
    var acceleration = EmptyPositionValue()
    var configurator = EmptyConfigurator()

    fun setIdleMode(mode: IdleMode) {}

    fun setNeutralMode(value: NeutralModeValue) {}

    fun setControl(request: ControlRequest) {}

    fun set(percent: Double) {}
}