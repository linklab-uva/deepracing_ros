

class PlannerParamNames:
    STATE = "state"
    GPU = "gpu"
    TIMESCALE = "timescale"
    CAR_WIDTH = "car_dims.width"
    CAR_LENGTH = "car_dims.length"

class RacelinePropagatorParamNames:
    STATE = PlannerParamNames.STATE
    TIMESCALE = PlannerParamNames.TIMESCALE
    GPU = PlannerParamNames.GPU
    NSEGMENTS = "Nsegments"
    PREDICTION_HORIZON = "prediction_horizon"
    PUBLISH_CAVSIM = "publish_cavsim"
    LAT_STDEV_RANGE = "stdev_range.lateral"
    LONG_STDEV_RANGE = "stdev_range.longitudinal"