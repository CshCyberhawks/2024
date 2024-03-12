package frc.robot.util

data class TelemetryToggles(
    var cannonTelemetry: Boolean = true,
    var trunkTelemetry: Boolean = true,
    var stateMachineTelemetry: Boolean = false
)