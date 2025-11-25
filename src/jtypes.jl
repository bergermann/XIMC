# type aliases for julia syntaxing

export DeviceId, Position

"""
    DeviceId = device_t

Stores the id of a motor as `int`
"""
const DeviceId = device_t
const DeviceEnumeration = device_enumeration_t
const Calibration = calibration_t
const DeviceInfo = device_information_t
const Status = status_t

"""
    Position = get_position_t

Stores the position of a motor.

# Fields
- `Position::Cint`: The step of the motor
- `uPosition::Cint` The micro step of the motor
- `EncPosition::long_t` The encoder position
"""
const Position = get_position_t
const Position(pos::Integer,upos::Integer) = Position(Cint(pos),Cint(upos))
const MoveSettings = move_settings_t
const EngineSettings = engine_settings_t
const EngineSettingsCalb = engine_settings_calb_t
const ControllerName  = controller_name_t
const StageName = stage_name_t
const StageInfo = stage_information_t
