function sysCall_init()
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    v = sim.getInt32Parameter(sim.intparam_program_version)
    if (v < 20413) then
        sim.displayDialog('Warning', 'The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!', sim.dlgstyle_ok, false, '', nil, { 0.8, 0, 0, 0, 0, 0 })
    end

    -- Detatch the manipulation sphere:
    targetObj = sim.getObjectHandle('Quadricopter_target')
    sim.setObjectParent(targetObj, -1, true)

    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example

    d = sim.getObjectHandle('Quadricopter_base')

    particlesAreVisible = sim.getScriptSimulationParameter(sim.handle_self, 'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree, 'particlesAreVisible', tostring(particlesAreVisible))
    simulateParticles = sim.getScriptSimulationParameter(sim.handle_self, 'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree, 'simulateParticles', tostring(simulateParticles))

    propellerScripts = { -1, -1, -1, -1 }
    for i = 1, 4, 1 do
        propellerScripts[i] = sim.getScriptHandle('Quadricopter_propeller_respondable' .. i)
    end
    heli = sim.getObjectAssociatedWithScript(sim.handle_self)

    particlesTargetVelocities = { 0, 0, 0, 0 }

    pParam = 2
    iParam = 0
    dParam = 0
    vParam = -2

    cumul = 0
    lastE = 0
    pAlphaE = 0
    pBetaE = 0
    psp2 = 0
    psp1 = 0

    prevEuler = 0

    fakeShadow = sim.getScriptSimulationParameter(sim.handle_self, 'fakeShadow')
    if (fakeShadow) then
        shadowCont = sim.addDrawingObject(sim.drawing_discpoints + sim.drawing_cyclic + sim.drawing_25percenttransparency + sim.drawing_50percenttransparency + sim.drawing_itemsizes, 0.2, 0, -1, 1)
    end

    -- Prepare 2 floating views with the camera views:
    floorCam = sim.getObjectHandle('Quadricopter_floorCamera')
    frontCam = sim.getObjectHandle('Quadricopter_frontCamera')
    floorView = sim.floatingViewAdd(0.9, 0.9, 0.2, 0.2, 0)
    frontView = sim.floatingViewAdd(0.7, 0.9, 0.2, 0.2, 0)
    sim.adjustView(floorView, floorCam, 64)
    sim.adjustView(frontView, frontCam, 64)


    --
    vector_h = sim.getObjectHandle('vector')
    testVector_h = sim.getObjectHandle('testVector')
    visionSensor_h = sim.getObjectHandle('Vision_sensor')
    payload_h = sim.getObjectHandle('Cuboid')

    if simROS then
        imu_handle = sim.getObjectHandle('IMU_link')
        targetSub = simROS.subscribe('/cmd_vel', 'geometry_msgs/Twist', 'target_cb')
        targetSub = simROS.subscribe('/testVector', 'geometry_msgs/Twist', 'testVector_cb')

        Gyro_pub = simROS.advertise('/imu', 'sensor_msgs/Imu')
        simROS.publisherTreatUInt8ArrayAsString(Gyro_pub)
        payload_pub = simROS.advertise('/payload', 'geometry_msgs/Twist')
        simROS.publisherTreatUInt8ArrayAsString(payload_pub)

        Imu_data = {}
        gyroCommunicationTube = sim.tubeOpen(0, 'gyroData' .. sim.getNameSuffix(nil), 1)
        accelCommunicationTube = sim.tubeOpen(0, 'accelerometerData' .. sim.getNameSuffix(nil), 1)
    end

    errorX = 0
    errorY = 0
    errorZ = 0

end

function target_cb(msg)
    -- Left motor speed subscriber callback
    targetPos = sim.getObjectPosition(targetObj, -1)
    v = { msg.linear.x, msg.linear.y, 0 }
    targetPos[1] = targetPos[1] + msg.linear.x / 500
    targetPos[2] = targetPos[2] + msg.linear.y / 500
    --sim.setObjectPosition(targetObj,-1,targetPos)
    --print(v)

    --512x512 resolution
    x = msg.linear.x * 1 / 512--212
    y = msg.linear.y * 1 / 512--220
    det = math.sqrt(x ^ 2 + y ^ 2 + 1)
    det = det * 1 / 2.81122
    --print(x)
    vector_pos = { -x / det, -y / det, 1 / det }
    --print(vector_pos)
    sim.setObjectPosition(vector_h, sim.handle_parent, vector_pos)

end

function testVector_cb(msg)
    --visionSensor_pos=sim.getObjectPosition(visionSensor_h,-1)
    --visionSensor_pos[1] = visionSensor_pos[1]+msg.linear.x
    --visionSensor_pos[2] = visionSensor_pos[2]-msg.linear.y
    --visionSensor_pos[3] = visionSensor_pos[3]-msg.linear.z
    --sim.setObjectPosition(testVector_h,-1,visionSensor_pos)
    errorX = msg.linear.x
    errorY = msg.linear.y
    errorZ = msg.linear.z

end

function sysCall_cleanup()
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(floorView)
    sim.floatingViewRemove(frontView)
end

function sysCall_actuation()
    s = sim.getObjectSizeFactor(d)

    pos = sim.getObjectPosition(d, -1)
    if (fakeShadow) then
        itemData = { pos[1], pos[2], 0.002, 0, 0, 1, 0.2 * s }
        sim.addDrawingObjectItem(shadowCont, itemData)
    end


    -- Vertical control:
    targetPos = sim.getObjectPosition(targetObj, -1)
    pos = sim.getObjectPosition(d, -1)
    l = sim.getVelocity(heli)
    e = (targetPos[3] - pos[3])
    cumul = cumul + e
    pv = pParam * e
    --5.335=kmg->k=5.335/1/9.8=0.544387755
    thrust = 5.335 + pv + iParam * cumul + dParam * (e - lastE) + l[3] * vParam - 0.544387755*1*errorZ
    lastE = e

    -- Horizontal control:
    sp = sim.getObjectPosition(targetObj, d)
    m = sim.getObjectMatrix(d, -1)
    vx = { 1, 0, 0 }
    vx = sim.multiplyVector(m, vx)
    vy = { 0, 1, 0 }
    vy = sim.multiplyVector(m, vy)
    --m[12]->z height
    alphaE = (vy[3] - m[12])
    alphaCorr = 0.25 * alphaE + 2.1 * (alphaE - pAlphaE)
    betaE = (vx[3] - m[12])
    betaCorr = -0.25 * betaE - 2.1 * (betaE - pBetaE)
    pAlphaE = alphaE
    pBetaE = betaE
    alphaCorr = alphaCorr + sp[2] * 0.005 + 1 * (sp[2] - psp2) - 1*errorY
    betaCorr = betaCorr - sp[1] * 0.005 - 1 * (sp[1] - psp1) - 1*errorX
    psp2 = sp[2]
    psp1 = sp[1]

    -- Rotational control:
    euler = sim.getObjectOrientation(d, targetObj)
    rotCorr = euler[3] * 0.1 + 2 * (euler[3] - prevEuler)
    prevEuler = euler[3]

    -- Decide of the motor velocities:
    particlesTargetVelocities[1] = thrust * (1 - alphaCorr + betaCorr + rotCorr)
    particlesTargetVelocities[2] = thrust * (1 - alphaCorr - betaCorr - rotCorr)
    particlesTargetVelocities[3] = thrust * (1 + alphaCorr - betaCorr + rotCorr)
    particlesTargetVelocities[4] = thrust * (1 + alphaCorr + betaCorr - rotCorr)

    -- Send the desired motor velocities to the 4 rotors:
    for i = 1, 4, 1 do
        sim.setScriptSimulationParameter(propellerScripts[i], 'particleVelocity', particlesTargetVelocities[i])
    end

end

function sysCall_sensing()
    if simROS then
        quaternion = sim.getObjectQuaternion(imu_handle, -1)
        accele_data = sim.tubeRead(accelCommunicationTube)
        gyro_data = sim.tubeRead(gyroCommunicationTube)
        if (accele_data and gyro_data) then
            acceleration = sim.unpackFloatTable(accele_data)
            angularVariations = sim.unpackFloatTable(gyro_data)
            Imu_data['orientation'] = { x = quaternion[1], y = quaternion[2], z = quaternion[3], w = quaternion[4] }
            Imu_data['header'] = { seq = 0, stamp = sim.getSystemTime(), frame_id = "imu_link" }
            Imu_data['linear_acceleration'] = { x = acceleration[1], y = acceleration[2], z = -acceleration[3] }
            Imu_data['angular_velocity'] = { x = angularVariations[1], y = angularVariations[2], z = angularVariations[3] }

            --simROS.sendTransform(getTransformStamped(imu_handle,'imu_link',base_handle,'base_link'))
            simROS.publish(Gyro_pub, Imu_data)
        end

        payload_pos=sim.getObjectPosition(payload_h,visionSensor_h)
        payload_data={}
        payload_data[linear]={x=payload_pos[1],y=payload_pos[2],z=payload_pos[3]}
        simROS.publish(payload_pub, payload_data)
    end
end