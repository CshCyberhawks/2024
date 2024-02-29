package frc.robot.constants

import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig

object PathPlannerLibConstants {
    // TODO: figure out if we need to change anything here
    val replanningConfig = ReplanningConfig()

    // TODO: tune; values are bad
    var translationPID = PIDConstants(5.0, 0.0, 0.0)
    val rotationPID = PIDConstants(0.0, 0.0)

}