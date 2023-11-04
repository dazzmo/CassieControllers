#ifndef INCLUDE_CONTROLLERS_OUTPUT_PRESCALE_HPP
#define INCLUDE_CONTROLLERS_OUTPUT_PRESCALE_HPP

class OutputPrescale {
   public:
    enum class PreScaleType {
        RAMP_UP,
        RAMP_DOWN,
        UNITY
    };

   private:
    double ramp_tau_ = 1.0;
    double t_ramp_start_ = 0.0;

    PreScaleType u_prescale_type_;

    double RampUp(void);
    double RampDown(void);
};

#endif /* INCLUDE_CONTROLLERS_OUTPUT_PRESCALE_HPP */
