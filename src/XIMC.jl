module XIMC

export infoXIMC, setupDevices, setBindyKey,
    requestDevices, closeDevice, closeDevices, checkOrdering, 
    commandMove, commandWaitForStop,
    getSpeed, setSpeed,
    getPos, getPosition, getPosition!,
    isSamePosition, isGreaterEqPosition, isSmallerEqPosition,
    isGreaterPosition, isSmallerPosition,
    ==, >=, <=, >, <,
    infoDevice, infoMoveSettings, infoPosition, infoSerial, infoSpeed, infoStage, infoStatus

using StringViews
using Printf

include("cenums.jl")
include("ctypes.jl")
include("jtypes.jl")
include("libximc_api.jl")
include("utilities.jl")

"""
    getDeviceInformation(device::DeviceId)

TBD
"""
function getDeviceInformation(device::DeviceId)
    deviceinfo = device_information_t()

    result = get_device_information(device,deviceinfo)

    if result != 0
        error("Result error: $result")
    end

    return deviceinfo
end

function getStageName(device::DeviceId; returnstruct::Bool=false)
    name = stage_name_t()

    result = get_stage_name(device,name)

    if result != 0
        error("Result error: $result")
    end

    if returnstruct
        return result, name
    end

    return String(name.PositionerName)
end

function getStageNameEnumerate(devenum::Ptr{DeviceEnumeration},idx::Integer)
    name = stage_name_t()

    result = get_enumerate_device_stage_name(devenum,idx-1,name)

    if result != 0
        error("Result error: $result")
    end

    return name
end

function getStageInformation(device::DeviceId)
    stageinfo = stage_information_t()

    result = get_stage_information(device,stageinfo)

    if result != 0
        error("Result error: $result")
    end

    return stageinfo
end

function getStatus(device::DeviceId)
    status = status_t()

    result = get_status(device,status)

    if result != 0
        error("Result error: $result")
    end

    return status
end

function getStatus!(status::Status,device::DeviceId)
    result = get_status(device,status)

    if result != 0
        error("Result error: $result")
    end
end

"""
    getPosition(device::DeviceId)

Return the current position of specified motor in steps.
"""
function getPosition(device::DeviceId)
    pos = get_position_t()

    result = get_position(device,pos)

    if result != 0
        error("Result error: $result")
    end

    return pos
end

"""
    getPosition!(pos::Position, device::DeviceId)

Store the current position of specified motor into `pos`.
"""
function getPosition!(pos::Position,device::DeviceId)
    result = get_position(device,pos)

    if result != 0
        error("Result error: $result")
    end
end

const getPos! = getPosition!
const getPos = getPosition

"""
    getPosition(devices::Vector{DeviceId}; fmt::Type=Matrix)

Return the current position of multiple motors.

The return type can be given by `fmt`:

- `Matrix`: (#devices, 3)-Matrix
    ,where each row corresponds to the motor in devices and contains
    [Position, uPosition, EncPosition]

- `Vector`: Vector of `Tuple{Int,Int}` of length #devices, where the tuple is
    (Position, uPosition)

The default is `Matrix`.
"""
function getPosition(devices::Vector{DeviceId}; fmt::Type=Matrix)   # fmt = Vector or Matrix
    if fmt == Matrix
        P = Matrix{Int64}(undef,length(devices),3)
    elseif fmt == Vector
        P = Vector{Tuple{Int,Int}}(undef,length(devices))
    else
        error("Unsupported output format.")
    end

    for i in eachindex(devices)
        p = getPosition(devices[i])
        if fmt == Matrix
            P[i,1] = copy(p.Position)
            P[i,2] = copy(p.uPosition)
            P[i,3] = copy(p.EncPosition)
        elseif fmt == Vector
            P[i] = (p.Position,p.uPosition)
        end
    end

    return P
end

"""
    getPosition(devices::DeviceId...; fmt::Type=Matrix)
"""
function getPosition(devices::DeviceId...; fmt::Type=Matrix)
    if fmt == Matrix
        P = Matrix{Int64}(undef,length(devices),3)
    elseif fmt == Vector
        P = Vector{Position}(undef,length(devices))
    else
        error("Unsupported output format.")
    end

    for i in eachindex(devices)
        p = getPosition(devices[i])
        if fmt == Matrix
            P[i,1] = copy(p.Position)
            P[i,2] = copy(p.uPosition)
            P[i,3] = copy(p.EncPosition)
        elseif fmt == Vector
            P[i] = copy(p)
        end
    end

    return P
end

"""
    function getPosition(devices::Vector{DeviceId},
        cal::Dict{String,Tuple{Symbol,Float64}};
        outputunit=:m)
        
    Return the position in given unit instead of steps.
"""
function getPosition(devices::Vector{DeviceId},
        cal::Dict{String,Tuple{Symbol,Float64}};
        outputunit=:m)
    P = Vector{Float64}(undef,length(devices))

    for i in eachindex(devices)
        p = getPosition(devices[i])
        P[i] = steps2x(p.Position,p.uPosition; outputunit=outputunit,
            cal=cal[getStageName(devices[i])])
    end

    return P
end

"""
    isSamePosition(x::Position, y::Position)

Returns wether `x` and `y` are the same Position.
"""
function isSamePosition(x::Position, y::Position)
    return (x.Position == y.Position) && (x.uPosition == y.uPosition)
end

"""
    isGreaterEqPosition(x::Position, y::Position)

Returns `x` >= `y`.
Note, that you can use the operator itself (eg. `x >= y`).
"""
function isGreaterEqPosition(x::Position, y::Position)
    return (x.Position > y.Position) || ((x.Position == y.Position) && (x.uPosition >= y.uPosition))
end

"""
    isSmallerEqPosition(x::Position, y::Position)

Returns `x` <= `y`.
Note, that you can use the operator itself (eg. `x <= y`).
"""
function isSmallerEqPosition(x::Position, y::Position)
    return (x.Position < y.Position) || ((x.Position == y.Position) && (x.uPosition <= y.uPosition))
end

"""
    isGreaterPosition(x::Position, y::Position)

Returns `x` > `y`.
Note, that you can use the operator itself (eg. `x > y`).
"""
function isGreaterPosition(x::Position, y::Position)
    return (x.Position > y.Position) || ((x.Position == y.Position) && (x.uPosition > y.uPosition))
end

"""
    isSmallerPosition(x::Position, y::Position)

Returns `x` < `y`.
Note, that you can use the operator itself (eg. `x < y`).
"""
function isSmallerPosition(x::Position, y::Position)
    return (x.Position < y.Position) || ((x.Position == y.Position) && (x.uPosition < y.uPosition))
end

"""
    const ==(x::Position, y::Position) = isSamePosition(x, y)
"""
const Base.:(==)(x::Position, y::Position) = isSamePosition(x, y)


"""
    const >=(x::Position, y::Position) = isGreaterEqPosition(x, y)
"""
const Base.:>=(x::Position, y::Position) = isGreaterEqPosition(x, y)

"""
    const <=(x::Position, y::Position) = isSmallerEqPosition(x, y)
"""
const Base.:<=(x::Position, y::Position) = isSmallerEqPosition(x, y)

"""
    const >(x::Position, y::Position) = isGreaterPosition(x, y)
"""
const Base.:>(x::Position, y::Position) = isGreaterPosition(x, y)

"""
    const <(x::Position, y::Position) = isSmallerPosition(x, y)
"""
const Base.:<(x::Position, y::Position) = isSmallerPosition(x, y)

function commandLeft(device::DeviceId; info=false)
    info && println("\nMoving left")

    result = command_left(device)

    info && println("\nResult: "*StringView(result))

    return result
end


"""
    commandMove(device::DeviceId, pos::Real, upos::Real; info=false)

Move the motor to the Position given by the steps `pos` and micro steps `upos`.
If `info=true` the return is printed to the console.
"""
function commandMove(device::DeviceId,pos::Int32,upos::Int32; info=false)
    info && println("\nGoing to $pos, $upos")

    result = command_move(device,pos,upos)

    info && println("\nResult: $result")

    return result
end

const commandMove(device::DeviceId,pos::Real,upos::Real; info=false) = commandMove(device,Int32(pos),Int32(upos); info=info)


"""
    commandMove(device::DeviceId,pos::Position; info=false)
"""
function commandMove(device::DeviceId,pos::Position; info=false)
    info && println("\nGoing to $pos, $upos")

    result = command_move(device,pos.Position,pos.uPosition)

    info && println("\nResult: $result")

    return result
end


"""
    commandMove(devices::Vector{DeviceId},positions::Vector{Tuple{Int,Int}})

Move multiple motors to the given positions.
"""
function commandMove(devices::Vector{DeviceId},positions::Vector{Tuple{Int,Int}})
    if length(devices) != length(positions)
        error("Device number and position number don't match.")
    end
    
    for i in eachindex(devices)
        commandMove(devices[i],positions[i][1],positions[i][2])
    end
end

"""
    commandMove(devices::Vector{DeviceId},positions::Matrix{Int32})
"""
function commandMove(devices::Vector{DeviceId},positions::Matrix{Int32})
    if size(positions,1) != length(devices) || size(positions,2) != 2
        error("Dimension mismatch: ndevices x 2 required.")
    end

    for i in 1:size(positions,1)
        commandMove(devices[i],positions[i,1],positions[i,2])
    end
end

"""
    commandMove(device::DeviceId,x::Real,cal::Dict{String,Tuple{Symbol,Float64}}; info=false,inputunit=:mm)

Move the motor using specified unit instead of steps.
"""
function commandMove(device::DeviceId,x::Real,cal::Dict{String,Tuple{Symbol,Float64}}; info=false,inputunit=:mm)
    p = x2steps(x; inputunit=inputunit,cal=cal[getStageName(device)])

    info && println("\n Going to $x$inputunit = $(p[1]), $(p[2])")

    result = command_move(device,p[1],p[2])

    info && println("\nResult: $result")

    return result
end

"""
    commandMove(devices::Vector{DeviceId},x::Vector{<:Real},cal::Dict{String,Tuple{Symbol,Float64}}; info=false,inputunit=:mm)

Move multiplied motors using specified unit instead of steps.
"""
function commandMove(devices::Vector{DeviceId},x::Vector{<:Real},cal::Dict{String,Tuple{Symbol,Float64}}; info=false,inputunit=:mm)
    if length(devices) != length(x)
        error("Amount of values don't match.")
    end

    for i in eachindex(devices)
        p = x2steps(x[i]; inputunit=inputunit,cal=cal[getStageName(devices[i])])

        info && println("\n D$i going to $x$inputunit = $(p[1]), $(p[2])")

        command_move(devices[i],p[1],p[2])
    end
end




"""
    commandWaitForStop(device::DeviceId; interval::UInt32=0x00000a,info::Bool=false)

Waiting for the specified motor to stop moving.

- `interval` specifies how many milliseconds the function waits to check again
    whether the motor is still moving.
"""
function commandWaitForStop(device::DeviceId; interval::UInt32=0x00000a,info::Bool=false)
    # info && println("\nWaiting for stop")

    result = command_wait_for_stop(device,interval)

    # info && println("\nResult: "*StringView(result))

    # return result
end


"""
    commandWaitForStop(devices::Vector{DeviceId}; interval::UInt32=0x00000a,info::Bool=false)

Waiting for multiple motors to stop moving.
"""
function commandWaitForStop(devices::Vector{DeviceId}; interval::UInt32=0x00000a,info::Bool=false)
    for i in eachindex(devices)
        commandWaitForStop(devices[i]; interval=interval,info=info)
    end
end

function enumerateDevices(flags,hints::Base.CodeUnits{UInt8, String})
    enum = enumerate_devices(flags,hints)

    return enum
end

function getControllerName(device::DeviceId)
    controller = controller_name_t()

    result = get_controller_name(device,controller)

    if result != 0
        error("Result error: $result")
    end

    return controller
end

function getEnumerateDeviceControllerName(devenum::Ptr{DeviceEnumeration},ind::Int32)
    controller = controller_name_t()

    result = get_enumerate_device_controller_name(devenum,ind-1,controller)

    if result != 0
        error("Result error: $result")
    end

    return controller
end

const getEnumerateDeviceControllerName(devenum::Ptr{DeviceEnumeration},ind::Real) = getEnumerateDeviceControllerName(devenum,Int32(ind))

function getDeviceCount(devenum::Ptr{DeviceEnumeration}; info=false)
    count = get_device_count(devenum)

    info && println("Device count: $count")

    return count
end

function getDeviceName(devenum::Ptr{DeviceEnumeration},ind::Integer)
    get_device_name(devenum,ind-1)
end

"""
    openDevice(uri)

Open the connection to the motor with the identifier `uri`.
"""
function openDevice(uri)
    open_device(uri)
end

function openDevices(enumnames::Vector{Cstring},stagenames::Dict{String,Int})
    if length(enumnames) < 1
        error("No devices to open!")
    end

    D = [openDevice(en) for en in enumnames]

    S = [String(getStageName(d)) for d in D]

    return D[sortperm([get(stagenames,s,0) for s in S])]
end

function checkOrdering(devices::Vector{DeviceId},stagenames::Dict{String,Int})
    # if length(devices) != length(stagenames)
    #     error("Unequal amounts of values.")
    # end

    @printf "%3s | %-17s\n" "D" "Stage names"
    for i in eachindex(devices)
        @printf "%3.0f | %-17s\n" i getStageName(devices[i])

        if stagenames[getStageName(devices[i])] != i
            error("Names did not match with ordering.")
        end
    end
end

"""
    closeDevice(device::DeviceId)

Close the connection to the motor.
"""
function closeDevice(device::DeviceId)
    close_device(device)
end

"""
    closeDevice(device::DeviceId)

Close the connection to multiple motors.
"""
function closeDevice(args::DeviceId...)
    for arg in args
        close_device(arg)
    end
end

"""
    closeDevices(args::DeviceId...) = closeDevice(args...)
"""
const closeDevices(args::DeviceId...) = closeDevice(args...)

# """
#     const closeDevices(devices::Vector{DeviceId}) = closeDevice.(devices)
# """
const closeDevices(devices::Vector{DeviceId}) = closeDevice.(devices)

function serialNumber(device::DeviceId)
    error("Not working yet.")

    serial = Ref(Cuint(0))

    result = get_serial_number(device,serial)

    if result != 0
        error("Result error: $result")
    end

    return serial
end

function getMoveSettings(device::DeviceId)
    mvst = move_settings_t()

    result = get_move_settings(device,Ref(mvst))

    if result != 0
        error("Result error: $result")
    end

    return mvst
end


"""
    getSpeed(device::DeviceId)

Returns a tuple `(Speed, uSpeed)` of the speed in steps/s.
"""
function getSpeed(device::DeviceId)
    mvst = move_settings_t()

    result = get_move_settings(device,Ref(mvst))

    if result != 0
        error("Result error: $result")
    end

    return (mvst.Speed, mvst.uSpeed)
end

function resetMoveSettings(device::DeviceId)
    mvst = move_settings_t()

    result = get_move_settings(device,Ref(mvst))

    if result != 0
        error("Result error: $result")
    end

    return mvst
end

function setMoveSettings(device::DeviceId,mvst::MoveSettings)
    result = set_move_settings(device,Ref(mvst))

    return result
end

"""
    setSpeed(device::DeviceId,speed::Integer; info=false)

Set the speed of the motor in steps/s.
"""
function setSpeed(device::DeviceId,speed::Integer; info=false)
    info && println("\nSet speed")

    mvst = getMoveSettings(device)

    info && println("Original speed: "*string(mvst.Speed))

    mvst.Speed = speed

    result = setMoveSettings(device,mvst)

    info && println("Write command result: "*string(result))
end

"""
    setSpeed(device::DeviceId,speed::Integer, uSpeed::Integer; info=false)

Set the speed of the motor in steps/s and (micro steps)/s.
"""
function setSpeed(device::DeviceId,speed::Integer, uSpeed::Integer; info=false)
    info && println("\nSet speed")

    mvst = getMoveSettings(device)

    info && println("Original speed: "*string(mvst.Speed))

    mvst.Speed = speed
    mvst.uSpeed = uSpeed

    result = setMoveSettings(device,mvst)

    info && println("Write command result: "*string(result))
end

"""
    setSpeed(device::Vector{DeviceId}, speed::Integer; info=false)

Set the speed of multiple motors in steps/s.
"""
function setSpeed(device::Vector{DeviceId}, speed::Integer; info=false)
    for D in device
        setSpeed(D, speed; info)
    end
end

"""
    setSpeed(device::Vector{DeviceId}, speed::Integer; info=false)

Set the speed of multiple motors in steps/s and (micro steps)/s.
"""
function setSpeed(device::Vector{DeviceId}, speed::Integer, uSpeed::Integer; info=false)
    for D in device
        setSpeed(D, speed, uSpeed; info)
    end
end

function getEngineSettings(device::DeviceId)
    eng = engine_settings_t()

    result = get_engine_settings(device,Ref(eng))

    if result != 0
        error("Result error: $result")
    end

    return eng
end

function setEngineSettings(device::DeviceId,eng::EngineSettings)
    result = get_engine_settings(device,Ref(eng))

    return result
end

function setMicroStepMode(device::DeviceId;
        mode::MicrostepMode=MICROSTEP_MODE_FRAC_256,info=false)

    info && println("\nSet microstep mode to $mode")

    _, eng = getEngineSettings(device)

    eng.MicrostepMode = copy(mode)

    result = setEngineSettings(device,eng)

    info && println("Command result: "*String(result))

    return result
end





function msecSleep(msec::UInt32)
    msec_sleep(msec)
end

"""
    infoDevice(device::DeviceId)

Print basic information about the motor to the console.
"""
function infoDevice(device::DeviceId)
    deviceinfo = getDeviceInformation(device)

    println("- Device information for D$device -")
    println("Manufacturer:       "*String(deviceinfo.Manufacturer))
    println("ManufacturerId:     "*String(deviceinfo.ManufacturerId))
    println("ProductDescription: "*String(deviceinfo.ProductDescription))
    println("Major:   ",deviceinfo.Major)
    println("Minor:   ",deviceinfo.Minor)
    println("Release: ",deviceinfo.Release)
end

"""
    infoStage(device::DeviceId)

Print stage information about the motor to the console.
"""
function infoStage(device::DeviceId)
    stagename = getStageName(device)
    stageinfo = getStageInformation(device)

    println("- Stage information for D$device -")
    println("Stage name:   ",String(stagename.PositionerName))
    println("Manufacturer: ",String(stageinfo.Manufacturer))
    println("Part number:  ",String(stageinfo.PartNumber))
end

"""
    infoStatus(device::DeviceId)

Print status information about the motor to the console.
"""
function infoStatus(device::DeviceId)
    status = getStatus(device)

    println("- Status information for D$device -")
    println("Status.Ipwr:  " + StringView(status.Ipwr))
    println("Status.Upwr:  " + StringView(status.Upwr))
    println("Status.Iusb:  " + StringView(status.Iusb))
    println("Status.Flags: " + StringView(hex(status.Flags)))
end

"""
    infoPosition(device::DeviceId)

Print position of the motor to the console.
"""
function infoPosition(device::DeviceId)
    pos = getPosition(device)

    println("- Position information for D$device -")
    println(" Position: "*StringView(pos.Position))
    println("uPosition: "*StringView(pos.uPosition))
end

"""
    infoSerial(device::DeviceId)

Print serial number of the motor to the console.
"""
function infoSerial(device::DeviceId)
    serial = serialNumber(device)

    println("Serial: "*string(serial))
end

"""
    infoMoveSettings(device::DeviceId)

Print move settings of the motor to the console.
"""
function infoMoveSettings(device::DeviceId)
    mvst = getMoveSettings(device)

    println("- Move setting information for D$device -")
    for field in fieldnames(MoveSettings)
        println("$field: "*String(getfield(mvst,field)))
    end
end

"""
    infoSpeed(device::DeviceId)

Print speed settings of the motor to the console.
"""
function infoSpeed(device::DeviceId)
    speed = getSpeed(device)

    println("- Speed information for D$device -")
    println(" Speed:"*String(speed[1]))
    println("uSpeed:"*String(speed[2]))
end

"""
    infoXIMC()

Prints and returns the current version number of the XIMC library.
"""
function infoXIMC()
    buf = Vector{UInt8}(undef,32)

    ximc_version(buf)

    filter!(x->x!=0x00,buf)

    println("\nXIMC library version: "*StringView(buf))

    return buf
end

function setupDevices(probeflags::UInt32,enumhints::Base.CodeUnits{UInt8,String})
    devenum = enumerateDevices(probeflags,enumhints)
    devcount = getDeviceCount(devenum; info=true)

    if devcount <= 0
        println("No devices found.")

        return Nothing,Nothing,Nothing
    end

    enumnames = Array{Cstring}(undef,devcount)

    @printf "%+2s | %-32s | %-8s | %-8s | %-16s\n" "D" "Name" "Serial" "Port" "Stage"

    for i in 1:devcount
        enumname = getDeviceName(devenum,i)
        enumctrlname = getEnumerateDeviceControllerName(devenum,i)
        stagename = getStageNameEnumerate(devenum,i)
        enumnames[i] = enumname

        n = unsafe_string(enumname)
        sn = parse(Int,"0x"*split(n,'/')[end])
        ax = String(enumctrlname.ControllerName)
        stn = String(stagename.PositionerName)

        @printf "%2.0f | %-32s | %-8.0f | %-8s | %-16s\n" i n sn ax stn
    end

    return devcount, devenum, enumnames
end

"""
    requestDevices(address, stagenames...)

Open the connection to motors with given identifiers `stagenames`.
If successful, returns a `Vector{DeviceId}` of given motors in the same order.
"""
function requestDevices(address::String, stagenames...)
    address = codeunits("addr="*address)

    devcount, devenum, enumnames =
        setupDevices(ENUMERATE_PROBE | ENUMERATE_NETWORK,address);

    D = [openDevice(en) for en in enumnames]
    D_ = zeros(Int32,length(stagenames))

    N = getStageName.(D)

    for s in stagenames
        if !(s in N)
            error("Device $s not found.")
        end
    end

    for d in D
        n = getStageName(d)

        if !(n in stagenames)
            closeDevice(d)

            continue
        end

        i = findfirst(isequal(n),stagenames)

        D_[i] = d
    end

    return D_
end

end # module XIMC