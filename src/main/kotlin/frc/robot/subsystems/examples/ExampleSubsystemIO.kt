package frc.robot.subsystems.examples

import org.littletonrobotics.junction.AutoLog

interface ExampleSubsystemIO {
    @AutoLog
    object ExampleSubsystemIOInputs {
        /** Sets the arm config, must be called before other methods.  */
        fun setConfig(config: ExampleSubsystemConfig) {}

        /** Updates the set of loggable inputs.  */
        fun updateInputs(inputs: ExampleSubsystemIOInputs) {}
    }
}