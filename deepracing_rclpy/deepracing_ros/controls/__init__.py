

class PlannerParamNames:
    STATE = "state"
    GPU = "gpu"
    TIMESCALE = "timescale"

class RacelinePropagatorParamNames:
    STATE = PlannerParamNames.STATE
    TIMESCALE = PlannerParamNames.TIMESCALE
    GPU = PlannerParamNames.GPU
    NSEGMENTS = "Nsegments"
    PREDICTION_HORIZON = "prediction_horizon"
    PUBLISH_CAVSIM = "publish_cavsim"
    LAT_STDEV_RANGE = "stdev_range.lateral"
    LONG_STDEV_RANGE = "stdev_range.longitudinal"