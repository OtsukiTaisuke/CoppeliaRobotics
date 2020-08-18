-- This is the Lumibot principal control script. It is threaded
getLightSensors=function()
    data=sim.receiveData(0,'LUMIBOT_lightSens')
    if (data) then
        lightSens=sim.unpackFloatTable(data)
    end
    return lightSens
end

function sysCall_threadmain()
    -- Put some initialization code here:
    sim.setThreadSwitchTiming(200) -- We will manually switch in the main loop
    leftMotor=sim.getObjectHandle('lumibot_leftMotor')
    rightMotor=sim.getObjectHandle('lumibot_rightMotor')
    lightSensorAdjustment=sim.getObjectHandle('lumibot_lightSensorAdjustment')
    lumibot=sim.getObjectAssociatedWithScript(sim.handle_self)
    floorSensorScriptHandle=sim.getScriptAssociatedWithObject(sim.getObjectHandle('lumibot_floorSensor'))

    trailPersistence=sim.getScriptSimulationParameter(sim.handle_self,'trailPersistence')
    trailMaxSize=sim.getScriptSimulationParameter(sim.handle_self,'trailMaxSize')
    trailMinSize=sim.getScriptSimulationParameter(sim.handle_self,'trailMinSize')
    sim.setScriptSimulationParameter(floorSensorScriptHandle,'trailPersistence',trailPersistence)
    sim.setScriptSimulationParameter(floorSensorScriptHandle,'trailMaxSize',trailMaxSize)
    sim.setScriptSimulationParameter(floorSensorScriptHandle,'trailMinSize',trailMinSize)
    maxVelocity=sim.getScriptSimulationParameter(sim.handle_self,'maxVelocity')
    motorCalibrationError=sim.getScriptSimulationParameter(sim.handle_self,'motorCalibrationError')
    sensorCalibrationError=sim.getScriptSimulationParameter(sim.handle_self,'sensorCalibrationError')
    randomlyModulateCalibrationErrors=sim.getScriptSimulationParameter(sim.handle_self,'randomlyModulateCalibrationErrors')

    maxVel=maxVelocity*math.pi/180
    backModeStart=-10
    rotateModeStart=-10
    rotateModeDurationMax=4
    rotateModeDurationMin=2
    rotateModeLeft=true
    bumper=sim.getObjectHandle('lumibot_bumpSensor')
    math.randomseed(os.time()+lumibot*13)
    for i=1,1000,1 do math.random() end
    if (randomlyModulateCalibrationErrors) then
        err=1+(math.random()-0.5)*motorCalibrationError
        sensorError=sensorCalibrationError*(math.random()-0.5)
    else
        err=1+0.5*motorCalibrationError
        sensorError=sensorCalibrationError*0.5
    end
    -- Here we execute the regular thread code:
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        st=sim.getSimulationTime()
        velLeft=0
        velRight=0
        lightSens=getLightSensors()
        if lightSens then
        lightSens[1]=lightSens[1]+sensorError
        lightSens[3]=lightSens[3]-sensorError
        if lightSens then
            diff=(lightSens[3]-lightSens[1])*1/0.1
            if (math.abs(diff)>1) then diff=diff/math.abs(diff) end
            velRight=maxVel
            if (diff<0) then
                velRight=maxVel*(1+diff)
            end
            velLeft=maxVel
            if (diff>0) then
                velLeft=maxVel*(1-diff)
            end
            velRight=velRight*err
            velLeft=velLeft*(1/err)
            if (math.abs(velRight)+math.abs(velLeft)<maxVel*0.5) then
                velRight=maxVel*0.25
                velLeft=-maxVel*0.25
            end
        end
        if (st<rotateModeStart) then
            if (rotateModeLeft) then
                velLeft=-maxVel*0.5
                velRight=maxVel*0.5
            else
                velLeft=maxVel*0.5
                velRight=-maxVel*0.5
            end
        end
        if (st<backModeStart) then
            if (st<backModeStart-0.5) then
                velLeft=-maxVel*0.5
                velRight=-maxVel*0.5
            else
                tt=math.random()
                rotateModeStart=st+(rotateModeDurationMax*tt+rotateModeDurationMin*(1-tt))
                rotateModeLeft=(math.random()>0.5)
                backModeStart=-10
            end
        end
        if (st>rotateModeStart)and(st>backModeStart) then
            r,f=sim.readForceSensor(bumper)
            if (r==1) then
                tf=math.sqrt(f[1]*f[1]+f[2]*f[2])
                if (tf>0.3) then
                    backModeStart=st+1
                end
            end
        end
        sim.setJointTargetVelocity(leftMotor,velLeft)
        sim.setJointTargetVelocity(rightMotor,velRight)
        end
        sim.switchThread() -- Don't waste too much time in here (simulation time will anyway only change in next thread switch)
    end
end
