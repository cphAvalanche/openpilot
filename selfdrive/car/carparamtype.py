from typing import List, Union

# This file contains Python classes that correspond to the structs defined in the Cap'n Proto schema file "cereal/car.cpnp",
#  specifically the "CarParams" struct. 

# These classes are provided solely to assist with type hinting in Python code that interacts
#  with the "CarParams" struct. They are not intended to be used to actually unwrap any Cap'n Proto structs.

# To use these classes for type hinting, import them in your Python code with `from carparamtype import *`. 
# For example, in the file seldrive/car/honda/interface.py, the _get_params() function 
#  uses these classes to define parameters and return type of the function. 
# When using a variable with the assigned type carparamtype (in the example named ret), possible completions
#  and class variable types will be shown when typing ret..

class SafetyModel:
    silent = 0
    hondaNidec = 1
    toyota = 2
    elm327 = 3
    gm = 4
    hondaBoschGiraffe = 5
    ford = 6
    cadillac = 7
    hyundai = 8
    chrysler = 9
    tesla = 10
    subaru = 11
    gmPassive = 12
    mazda = 13
    nissan = 14
    volkswagen = 15
    toyotaIpas = 16
    allOutput = 17
    gmAscm = 18
    noOutput = 19
    hondaBosch = 20
    volkswagenPq = 21
    subaruLegacy = 22
    hyundaiLegacy = 23
    hyundaiCommunity = 24
    volkswagenMlb = 25
    hongqi = 26
    body = 27
    hyundaiCanfd = 28


class LateralPIDTuning:
    def __init__(self, kpBP: List[float], kpV: List[float], kiBP: List[float],
                 kiV: List[float], kf: float):
        self.kpBP = kpBP
        self.kpV = kpV
        self.kiBP = kiBP
        self.kiV = kiV
        self.kf = kf


class SteerControlType:
    torque = 0
    angle = 1
    curvature = 2


class TransmissionType:
    unknown = 0
    automatic = 1
    manual = 2
    direct = 3
    cvt = 4

class Ecu:
    eps = 0
    abs = 1
    fwdRadar = 2
    fwdCamera = 3
    engine = 4
    unknown = 5
    transmission = 8 # Transmission Control Module
    srs = 9
    gateway = 10 # can gateway
    hud = 11 # heads up display
    combinationMeter = 12 # instrument cluster
    electricBrakeBooster = 15
    shiftByWire = 16
    adas = 19    
    cornerRadar = 21
    # Toyota only
    dsu = 6
    apgs = 7

    # Honda only
    vsa = 13 # Vehicle Stability Assist
    programmedFuelInjection = 14

    # Chrysler only
    hcp = 18 # Hybrid Control Processor

    # Hyundai only
    vcu = 20 # Vehicle (Motor) Control Unit

    debug = 17

class CarFw:
    ecu: Ecu
    fwVersion: Data
    address: int
    subAddress: int
    responseAddress: int
    request: List[Data]
    brand: str
    bus: int
    logging: bool


class FingerprintSource:
    can = 0
    fw = 1
    fixed = 2


class NetworkLocation:
    fwdCamera = 0
    gateway = 1

class SafetyConfig:
    safetyModel: SafetyModel
    safetyParam: int
    safetyParamDEPRECATED: int
    safetyParam2DEPRECATED: int

class LongitudinalPIDTuning:
    kpBP: List[float]
    kpV: List[float]
    kiBP: List[float]
    kiV: List[float]
    kf: float
    deadzoneBP: List[float]
    deadzoneV: List[float]

class LateralParams:
    torqueBP: List[int]
    torqueV: List[int]

class LateralINDITuning:
    outerLoopGainBP: List[float]
    outerLoopGainV: List[float]
    innerLoopGainBP: List[float]
    innerLoopGainV: List[float]
    timeConstantBP: List[float]
    timeConstantV: List[float]
    actuatorEffectivenessBP: List[float]
    actuatorEffectivenessV: List[float]

    outerLoopGainDEPRECATED: float
    innerLoopGainDEPRECATED: float
    timeConstantDEPRECATED: float
    actuatorEffectivenessDEPRECATED: float

class LateralLQRTuning:
    scale: float
    ki: float
    dcGain: float

    a: List[float]
    b: List[float]
    c: List[float]

    k: List[float]
    l: List[float]

class LateralTorqueTuning:
    useSteeringAngle: bool
    kp: float
    ki: float
    friction: float
    kf: float
    steeringAngleDeadzoneDeg: float
    latAccelFactor: float
    latAccelOffset: float

class CarParamType():
    carName: str
    carFingerprint: str
    fuzzyFingerprint: bool

    notCar: bool # flag for non-car robotics platforms

    enableGasInterceptor: bool
    pcmCruise: bool # is openpilot's state tied to the PCM's cruise state?
    enableDsu: bool # driving support unit
    enableBsm: bool # blind spot monitoring
    flags: int      # flags for car specific quirks
    experimentalLongitudinalAvailable: bool

    minEnableSpeed: float
    minSteerSpeed: float
    safetyConfigs: List[SafetyConfig]
    alternativeExperience: int # panda flag for features like no disengage on gas

    # Car docs fields
    maxLateralAccel: float
    autoResumeSng: bool  # describes whether car can resume from a stop automatically

    # things about the car in the manual
    mass: float           # [kg] curb weight: all fluids no cargo
    wheelbase: float      # [m] distance from rear axle to front axle
    centerToFront: float  # [m] distance from center of mass to front axle
    steerRatio: float     # [] ratio of steering wheel angle to front wheel angle
    steerRatioRear: float # [] ratio of steering wheel angle to rear wheel angle (usually 0)

    # things we can derive
    rotationalInertia: float  # [kg*m2] body rotational inertia
    tireStiffnessFront: float # [N/rad] front tire coeff of stiff
    tireStiffnessRear: float  # [N/rad] rear tire coeff of stiff

    longitudinalTuning: LongitudinalPIDTuning
    lateralParams: LateralParams
    lateralTuning: Union[LateralPIDTuning, LateralINDITuning, LateralLQRTuning, LateralTorqueTuning]

    steerLimitAlert: bool
    steerLimitTimer: float # time before steerLimitAlert is issued

    steerLimitAlert: bool
    steerLimitTimer: float  # time before steerLimitAlert is issued

    vEgoStopping: float  # Speed at which the car goes into stopping state
    vEgoStarting: float  # Speed at which the car goes into starting state
    stoppingControl: bool  # Does the car allow full control even at lows speeds when stopping
    steerControlType: SteerControlType
    radarUnavailable: bool  # True when radar objects aren't visible on CAN or aren't parsed out
    stopAccel: float  # Required acceleration to keep vehicle stationary
    stoppingDecelRate: float  # m/s^2/s while trying to stop
    startAccel: float  # Required acceleration to get car moving
    startingState: bool  # Does this car make use of special starting state

    steerActuatorDelay: float  # Steering wheel actuator delay in seconds
    longitudinalActuatorDelayLowerBound: float  # Gas/Brake actuator delay in seconds, lower bound
    longitudinalActuatorDelayUpperBound: float  # Gas/Brake actuator delay in seconds, upper bound
    openpilotLongitudinalControl: bool  # is openpilot doing the longitudinal control?
    carVin: str  # VIN number queried during fingerprinting
    dashcamOnly: bool
    transmissionType: TransmissionType
    carFw: List[CarFw]

    radarTimeStep: float = 0.05  # time delta between radar updates, 20Hz is very standard
    fingerprintSource: FingerprintSource
    networkLocation: NetworkLocation  # Where Panda/C2 is integrated into the car's CAN network

    wheelSpeedFactor: float  # Multiplier on wheels speeds to computer actual speeds
