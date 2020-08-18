
function __setObjectPosition__(a,b,c)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.setObjectPosition(a,b,c)
end

function __getObjectPosition__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectPosition(a,b)
end
function __setObjectOrientation__(a,b,c)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.setObjectOrientation(a,b,c)
end
function __getObjectOrientation__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectOrientation(a,b)
end

-- This example script is non-threaded (executed at each simulation pass)

-- The functionality of this script (or parts of it) could be implemented

-- in an extension module (plugin) and be hidden. The extension module could

-- also allow connecting to and controlling the real robot.


if (sim_call_type==sim.syscb_init) then 
    --Prepare initial values and retrieve handles:
    irb140=sim.getObjectHandle('IRB140')
    tip=sim.getObjectHandle('IRB140_tip')
    targetSphere=sim.getObjectHandle('IRB140_manipulationSphere')
    targetSphereBase=sim.getObjectHandle('IRB140_manipulationSphereBase')
    armJoints={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        armJoints[i]=sim.getObjectHandle('IRB140_joint'..i)
    end
    ui=simGetUIHandle('IRB140_UI')
    simSetUIButtonLabel(ui,0,sim.getObjectName(irb140)..' user interface') -- Set the UI title (with the name of the current robot)
    ik1=sim.getIkGroupHandle('IRB140_undamped')
    ik2=sim.getIkGroupHandle('IRB140_damped')
    ikFailedReportHandle=-1

    initSizeFactor=sim.getObjectSizeFactor(irb140) -- only needed if we scale the robot up/down
    -- desired joint positions, and desired cartesian positions:
    desiredJ={0,0,0,0,-90*math.pi/180,0} -- when in FK mode
    for i=1,6,1 do
        sim.setJointPosition(armJoints[i],desiredJ[i])
    end
    desiredConf={0,0,0,0,0,0} -- when in IK mode
    currentConf={0,0,0,0,0,0} -- when in IK mode
    ikMinPos={-1.5*initSizeFactor,-1*initSizeFactor,-1*initSizeFactor}
    ikRange={2*initSizeFactor,2*initSizeFactor,1.75*initSizeFactor}
    -- We compute the initial position and orientation of the tip RELATIVE to the robot base (because the base is moving)
    initialTipPosRelative=__getObjectPosition__(tip,irb140)
    -- Before V-REP V2.5.1, "sim.getObjectOrientation" contained a bug when Euler angles were queried not absolutely
    -- So instead of using "initialTipOrientRelative=__getObjectOrientation__(tip,irb140)", we make it a little bit more complicated:
    m=sim.getObjectMatrix(tip,irb140)
    initialTipOrientRelative=sim.getEulerAnglesFromMatrix(m)

    movementMode=0 -- 0=FK, 1=IK through dialog, 2=IK through manipulation sphere
    maxJointVelocity=180*math.pi/180 
    maxPosVelocity=1.0*initSizeFactor
    maxOrientVelocity=45*math.pi/180
    previousS=initSizeFactor
end 

if (sim_call_type==sim.syscb_cleanup) then 
 
end 

if (sim_call_type==sim.syscb_actuation) then 
    -- s will scale a few values hereafter (has only an effect if the robot is scaled down/up)
    s=sim.getObjectSizeFactor(irb140) 
    if (s~=previousS) then
        f=s/previousS
        for i=1,3,1 do
            desiredConf[i]=desiredConf[i]*f

            currentConf[i]=currentConf[i]*f

            ikMinPos[i]=ikMinPos[i]*f

            ikRange[i]=ikRange[i]*f

            initialTipPosRelative[i]=initialTipPosRelative[i]*f
        end
        maxPosVelocity=maxPosVelocity*f
        previousS=s
    end
    maxJointVelocity=simGetUISlider(ui,9)*0.001*math.pi
    maxPosVelocity=simGetUISlider(ui,10)*0.001*initSizeFactor
    maxOrientVelocity=simGetUISlider(ui,10)*0.001*math.pi/2
    -- Now we check if we wanna move the robot through the green manipulation sphere:
    pos=sim.getObjectPosition(targetSphere,sim.handle_parent)
    -- Before V-REP V2.5.1, "sim.getObjectOrientation" contained a bug when Euler angles were queried not absolutely
    -- So instead of using "orient=__getObjectOrientation__(targetSphere,sim.handle_parent)", we make it a little bit more complicated:
    m=sim.getObjectMatrix(targetSphere,sim.handle_parent)
    euler=sim.getEulerAnglesFromMatrix(m)
    if (math.abs(pos[1])>0.0001)or(math.abs(pos[2])>0.0001)or(math.abs(pos[3])>0.0001)or
        (math.abs(euler[1])>0.0005)or(math.abs(euler[2])>0.0005)or(math.abs(euler[3])>0.0005) then
            movementMode=2 -- IK by sphere manipulation
            m=sim.getObjectMatrix(targetSphere,-1)
            sim.setObjectMatrix(targetSphereBase,-1,m)
            sim.setObjectPosition(targetSphere,sim.handle_parent,{0,0,0})
            __setObjectOrientation__(targetSphere,sim.handle_parent,{0,0,0})

            m=sim.getObjectMatrix(targetSphere,irb140)
            euler=sim.getEulerAnglesFromMatrix(m)
            desiredConf={m[4]-initialTipPosRelative[1],m[8]-initialTipPosRelative[2],m[12]-initialTipPosRelative[3],
                        euler[1]-initialTipOrientRelative[1],euler[2]-initialTipOrientRelative[2],euler[3]-initialTipOrientRelative[3]}
            for i=1,6,1 do 
                currentConf[i]=desiredConf[i]
            end
    end

    buttonID=simGetUIEventButton(ui)

    if (buttonID>=1001)and(buttonID<=1006) then -- we want to control the arm in FK mode!
        cyclic,interval=sim.getJointInterval(armJoints[buttonID-1000])
        desiredJ[buttonID-1000]=interval[1]+simGetUISlider(ui,buttonID)*0.001*interval[2]
        movementMode=0
    end

    if ((buttonID>=2001)and(buttonID<=2003)) then -- we want to control the arm in IK mode! (position only is handled here)
        desiredConf[buttonID-2000]=ikMinPos[buttonID-2000]+ikRange[buttonID-2000]*simGetUISlider(ui,buttonID)/1000
        movementMode=1
    end

    if ((buttonID==2004)or(buttonID==2006)) then -- we want to control the arm in IK mode! (orientation 1&3 only is handled here)
        desiredConf[buttonID-2000]=math.pi*(-1+2*simGetUISlider(ui,buttonID)/1000)
        movementMode=1
    end

    if (buttonID==2005) then -- we want to control the arm in IK mode! (orientation 2 only is handled here)
        desiredConf[buttonID-2000]=0.5*math.pi*(-1+2*simGetUISlider(ui,buttonID)/1000)
        movementMode=1
    end

    if movementMode==1 then
        -- We are in IK mode
        maxLinVariationAllowed=maxPosVelocity*sim.getSimulationTimeStep()
        maxAngVariationAllowed=maxOrientVelocity*sim.getSimulationTimeStep()
        deltaX={0,0,0,0,0,0}
        -- position:
        for i=1,3,1 do
            deltaX[i]=desiredConf[i]-currentConf[i]
            if (math.abs(deltaX[i])>maxLinVariationAllowed) then
                deltaX[i]=maxLinVariationAllowed*deltaX[i]/math.abs(deltaX[i]) -- we limit the variation to the maximum allowed
            end
        end

        -- orientation:
        for i=1,3,1 do
            deltaX[3+i]=desiredConf[3+i]-currentConf[3+i]
            -- Normalize delta to be between -pi and +pi (or -pi/2 and +pi/2 for beta):
            if (i==2) then
                rnge=math.pi
            else
                rnge=math.pi*2
            end

            deltaX[3+i]=math.fmod(deltaX[3+i],rnge)
            if (deltaX[3+i]<-rnge*0.5) then
                deltaX[3+i]=deltaX[3+i]+rnge
            else
                if (deltaX[3+i]>rnge*0.5) then
                    deltaX[3+i]=deltaX[3+i]-rnge
                end
            end
            if (math.abs(deltaX[3+i])>maxAngVariationAllowed) then
                deltaX[3+i]=maxAngVariationAllowed*deltaX[3+i]/math.abs(deltaX[3+i]) -- we limit the variation to the maximum allowed
            end
        end

        for i=1,6,1 do
            currentConf[i]=currentConf[i]+deltaX[i]
        end
        -- Normalize the orientation part to display normalized values:
        for i=1,3,1 do
            f=1
            if i==2 then f=0.5 end
            currentConf[3+i]=math.fmod(currentConf[3+i],math.pi*2*f)
            if (currentConf[3+i]<-math.pi*f) then 
                currentConf[3+i]=currentConf[3+i]+math.pi*2*f
            else 
                if (currentConf[3+i]>math.pi*f) then
                    currentConf[3+i]=currentConf[3+i]-math.pi*2*f
                end
            end
        end

        pos={0,0,0}
        orient={0,0,0}
        for i=1,3,1 do
            pos[i]=initialTipPosRelative[i]+currentConf[i]
            orient[i]=initialTipOrientRelative[i]+currentConf[3+i]
        end
        -- We set the desired position and orientation
        __setObjectPosition__(targetSphereBase,irb140,pos)
        __setObjectOrientation__(targetSphereBase,irb140,orient)
    end

    if (movementMode==1) or (movementMode==2) then
        if (sim.handleIkGroup(ik1)==sim.ikresult_fail) then
            -- the position/orientation could not be reached.
            sim.handleIkGroup(ik2) -- Apply a damped resolution method
            if (ikFailedReportHandle==-1) then -- We display a IK failure report message
                ikFailedReportHandle=sim.displayDialog("IK failure report","IK solver failed.",sim.dlgstyle_message,false,"",nil,{1,0.7,0,0,0,0})
            end
        else
            if (ikFailedReportHandle>=0) then
                sim.endDialog(ikFailedReportHandle) -- We close any report message about IK failure
                ikFailedReportHandle=-1
            end
        end
        -- Now update the desiredJ in case we switch back to FK mode:
        for i=1,6,1 do
            desiredJ[i]=sim.getJointPosition(armJoints[i])
        end
    end
    
    if movementMode==0 then    
        -- We are in FK mode
        currentJ={0,0,0,0,0,0}
        for i=1,6,1 do
            currentJ[i]=sim.getJointPosition(armJoints[i])
        end
        maxVariationAllowed=maxJointVelocity*sim.getSimulationTimeStep()
        for i=1,6,1 do
            delta=desiredJ[i]-currentJ[i]
            if (sim.getJointInterval(armJoints[i])) then
                -- Joint is cyclic, we go the fastest direction:
                if (delta>math.pi) then
                    delta=delta-math.pi*2
                end

                if (delta<-math.pi) then
                    delta=delta+math.pi*2
                end
            end

            if (math.abs(delta)>maxVariationAllowed) then
                delta=maxVariationAllowed*delta/math.abs(delta) -- we limit the variation to the maximum allowed
            end
            sim.setJointPosition(armJoints[i],currentJ[i]+delta)
        end
        -- Now make sure that everything is ok if we switch to IK mode:
        sim.setObjectPosition(targetSphereBase,-1,sim.getObjectPosition(tip,-1))
        sim.setObjectOrientation(targetSphereBase,-1,sim.getObjectOrientation(tip,-1))
        tipPosRel=__getObjectPosition__(tip,irb140)
        -- Before V-REP V2.5.1, "sim.getObjectOrientation" contained a bug when Euler angles were queried not absolutely
        -- So instead of using "tipOrientRel=__getObjectOrientation__(tip,irb140)", we make it a little bit more complicated:
        m=sim.getObjectMatrix(tip,irb140)

        tipOrientRel=sim.getEulerAnglesFromMatrix(m)

        desiredConf={tipPosRel[1]-initialTipPosRelative[1],tipPosRel[2]-initialTipPosRelative[2],tipPosRel[3]-initialTipPosRelative[3],
                        tipOrientRel[1]-initialTipOrientRelative[1],tipOrientRel[2]-initialTipOrientRelative[2],tipOrientRel[3]-initialTipOrientRelative[3]}
        for i=1,6,1 do 
            currentConf[i]=desiredConf[i]
        end
        -- Close any IK warning dialogs:
        if (ikFailedReportHandle>=0) then
            sim.endDialog(ikFailedReportHandle) -- We close any report message about IK failure
            ikFailedReportHandle=-1
        end
    end
    -- Now update the user interface:
    -- First the FK part, text boxes:
    for i=1,6,1 do
        simSetUIButtonLabel(ui,1010+i,string.format("%.1f",sim.getJointPosition(armJoints[i])*180/math.pi))
    end
    -- Then the FK part, sliders, based on the target joint position if in FK mode, or based on the current joint position if in IK mode:
    for i=1,6,1 do
        cyclic,interval=sim.getJointInterval(armJoints[i])
        if (movementMode~=0) then
            simSetUISlider(ui,1000+i,1000*(sim.getJointPosition(armJoints[i])-interval[1])/interval[2])
        else
            simSetUISlider(ui,1000+i,1000*(desiredJ[i]-interval[1])/interval[2])
        end
    end
    -- Now the IK part:
    -- First the text boxes:
    -- Linear:
    for i=1,3,1 do
        str=string.format("%.3f",currentConf[i])
        if (str=='-0.000') then
            str='0.000' -- avoid having the - sign appearing and disappearing when 0
        end
        simSetUIButtonLabel(ui,2010+i,str)
    end
    -- Angular:
    for i=1,3,1 do
        str=string.format("%.1f",currentConf[3+i]*180/math.pi)
        if (str=='-0.0') then
            str='0.0' -- avoid having the - sign appearing and disappearing when 0
        end
        simSetUIButtonLabel(ui,2013+i,str)
    end
    -- Now the sliders, based on the desired configuration if in IK mode, or based on the current tip configuration if in FK mode:
    -- Linear:
    for i=1,3,1 do
        if (movementMode~=0) then
            simSetUISlider(ui,2000+i,1000*(desiredConf[i]-ikMinPos[i])/ikRange[i])
        else
            simSetUISlider(ui,2000+i,1000*(currentConf[i]-ikMinPos[i])/ikRange[i])
        end
    end
    -- Angular:
    if (movementMode~=0) then
        simSetUISlider(ui,2004,1000*(desiredConf[4]+math.pi)/(2*math.pi))
        simSetUISlider(ui,2005,1000*(desiredConf[5]+math.pi*0.5)/math.pi)
        simSetUISlider(ui,2006,1000*(desiredConf[6]+math.pi)/(2*math.pi))
    else
        simSetUISlider(ui,2004,1000*(currentConf[4]+math.pi)/(2*math.pi))
        simSetUISlider(ui,2005,1000*(currentConf[5]+math.pi*0.5)/math.pi)
        simSetUISlider(ui,2006,1000*(currentConf[6]+math.pi)/(2*math.pi))
    end
end 

