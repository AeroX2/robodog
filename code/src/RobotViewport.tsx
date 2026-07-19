import { forwardRef, useEffect, useImperativeHandle, useMemo, useRef, useState } from 'react'
import type { MutableRefObject } from 'react'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { Grid, Line, OrbitControls, TransformControls } from '@react-three/drei'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import * as THREE from 'three'
import URDFLoader from 'urdf-loader'
import {
  applySpringPose,
  applyMinimumPose,
  applyStandingPose,
  findObject,
  LEGS,
  solveLegIK,
  type IKResult,
  type LegId,
  type URDFRobotLike,
} from './kinematics'

export type ViewerTelemetry = IKResult & {
  target: [number, number, number]
  loaded: boolean
  solving: boolean
  message?: string
}

export type GaitSettings = {
  mode: 'pose' | 'walk'
  playing: boolean
  speed: number
  physics: boolean
}

export type GaitTelemetry = {
  phase: number
  swingLeg: LegId
  contacts: number
  bodyOffset: number
  roll: number
  pitch: number
  distance: number
  speed: number
}

export type ViewerHandle = {
  setTarget: (target: [number, number, number]) => void
  homeTarget: () => void
  resetPath: () => void
  resetPose: () => void
  resetCamera: () => void
}

type SceneProps = {
  leg: LegId
  gait: GaitSettings
  onTelemetry: (value: ViewerTelemetry) => void
  onGaitUpdate: (value: GaitTelemetry) => void
  apiRef: MutableRefObject<ViewerHandle | null>
}

export const GAIT_ORDER: LegId[] = ['front_left', 'back_right', 'front_right', 'back_left']
export const PATH_POINT_LABELS = ['Rear contact', 'Swing apex', 'Front contact', 'Stance midpoint']
const SWING_FRACTION = 0.18
const BODY_COM = new THREE.Vector3(0, -0.33, 1)
const GROUND_HEIGHT = 0.049
const PHYSICS_MASS = 12
const PHYSICS_GRAVITY = 9.81
const LEG_STIFFNESS = 760
const LEG_DAMPING = 48
const ANGULAR_INERTIA = 3.2
const ANGULAR_DAMPING = 5.5
const STATIC_COMPRESSION = PHYSICS_MASS * PHYSICS_GRAVITY / (4 * LEG_STIFFNESS)

const SUPPORT_MOUNTS: Record<LegId, { x: number; y: number }> = {
  front_left: { x: 0.331, y: -0.677 },
  front_right: { x: -0.331, y: -0.677 },
  back_left: { x: 0.331, y: 0.677 },
  back_right: { x: -0.331, y: 0.677 },
}

type PhysicsState = {
  z: number
  vz: number
  roll: number
  pitch: number
  rollVelocity: number
  pitchVelocity: number
}

function createPhysicsState(): PhysicsState {
  return { z: 0, vz: 0, roll: 0, pitch: 0, rollVelocity: 0, pitchVelocity: 0 }
}

function applyBodyTransform(robot: URDFRobotLike, physics: PhysicsState, distance: number) {
  robot.rotation.set(physics.roll, physics.pitch, 0)
  const rotatedCom = BODY_COM.clone().applyQuaternion(robot.quaternion)
  robot.position.copy(BODY_COM).sub(rotatedCom)
  robot.position.y -= distance
  robot.position.z += physics.z
  robot.updateMatrixWorld(true)
}

function resetBodyTransform(robot: URDFRobotLike) {
  robot.position.set(0, 0, 0)
  robot.rotation.set(0, 0, 0)
  robot.updateMatrixWorld(true)
}

function createDefaultPath(legId: LegId) {
  const outward = legId.endsWith('left') ? 0.055 : -0.055
  return [
    new THREE.Vector3(0, 0.11, 0),
    new THREE.Vector3(outward, 0, 0.12),
    new THREE.Vector3(0, -0.11, 0),
    new THREE.Vector3(0, 0, 0),
  ]
}

function createDefaultPaths() {
  return Object.fromEntries(
    (Object.keys(LEGS) as LegId[]).map((legId) => [legId, createDefaultPath(legId)]),
  ) as Record<LegId, THREE.Vector3[]>
}

function smoothSegment(start: THREE.Vector3, end: THREE.Vector3, t: number, target: THREE.Vector3) {
  const smooth = t * t * (3 - 2 * t)
  return target.lerpVectors(start, end, smooth)
}

function sampleFootPath(points: THREE.Vector3[], localPhase: number, target = new THREE.Vector3()) {
  if (localPhase < SWING_FRACTION) {
    const swing = localPhase / SWING_FRACTION
    return swing < 0.5
      ? smoothSegment(points[0], points[1], swing * 2, target)
      : smoothSegment(points[1], points[2], (swing - 0.5) * 2, target)
  }
  const stance = (localPhase - SWING_FRACTION) / (1 - SWING_FRACTION)
  return stance < 0.5
    ? smoothSegment(points[2], points[3], stance * 2, target)
    : smoothSegment(points[3], points[0], (stance - 0.5) * 2, target)
}

function CameraController({ controlsRef }: { controlsRef: MutableRefObject<OrbitControlsImpl | null> }) {
  const { camera } = useThree()
  useEffect(() => {
    camera.up.set(0, 0, 1)
    camera.updateProjectionMatrix()
  }, [camera])
  return <OrbitControls ref={controlsRef} makeDefault target={[0, -0.1, 0.38]} minDistance={2.2} maxDistance={9} />
}

function TargetMarker() {
  return (
    <group>
      <mesh>
        <sphereGeometry args={[0.075, 24, 18]} />
        <meshStandardMaterial color="#ff9f1c" emissive="#ff7a00" emissiveIntensity={0.5} roughness={0.35} />
      </mesh>
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <torusGeometry args={[0.13, 0.012, 10, 40]} />
        <meshBasicMaterial color="#ffd08a" transparent opacity={0.86} />
      </mesh>
      <pointLight color="#ff982f" intensity={0.7} distance={1.2} />
    </group>
  )
}

function RobotScene({ leg, gait, onTelemetry, onGaitUpdate, apiRef }: SceneProps) {
  const [robot, setRobot] = useState<URDFRobotLike | null>(null)
  const [loadError, setLoadError] = useState<string | null>(null)
  const [selectedPathPoint, setSelectedPathPoint] = useState(-1)
  const [selectedPathObject, setSelectedPathObject] = useState<THREE.Mesh | null>(null)
  const [pathRevision, setPathRevision] = useState(0)
  const targetRef = useRef<THREE.Group>(null)
  const controlsRef = useRef<OrbitControlsImpl | null>(null)
  const cameraTravelRef = useRef(0)
  const pathOffsetsRef = useRef<Record<LegId, THREE.Vector3[]>>(createDefaultPaths())
  const draggingRef = useRef(false)
  const activeLegRef = useRef(leg)
  const gaitTimeRef = useRef(0)
  const gaitHomesRef = useRef<Partial<Record<LegId, THREE.Vector3>>>({})
  const lastGaitPublishRef = useRef(0)
  const travelRef = useRef(0)
  const physicsRef = useRef<PhysicsState>(createPhysicsState())
  const contactMarkerRefs = useRef<Partial<Record<LegId, THREE.Group>>>({})
  const comMarkerRef = useRef<THREE.Group>(null)
  const pathEditorRef = useRef<THREE.Group>(null)
  const contactStatesRef = useRef<Record<LegId, boolean>>({
    front_left: true,
    front_right: true,
    back_left: true,
    back_right: true,
  })
  const targetPosition = useMemo(() => new THREE.Vector3(), [])
  const gaitTarget = useMemo(() => new THREE.Vector3(), [])
  const pathSample = useMemo(() => new THREE.Vector3(), [])
  const comWorldPosition = useMemo(() => new THREE.Vector3(), [])
  const visiblePathSamples = useMemo(() => {
    const points = pathOffsetsRef.current[leg]
    return Array.from({ length: 65 }, (_, index) => sampleFootPath(points, index / 64))
  }, [leg, pathRevision])

  const publish = (result: IKResult, target: THREE.Vector3, solving = false) => {
    onTelemetry({
      ...result,
      target: [target.x, target.y, target.z],
      loaded: true,
      solving,
    })
  }

  const placeTargetAtFoot = (activeRobot = robot, activeLeg = activeLegRef.current) => {
    if (!activeRobot || !targetRef.current) return
    const foot = findObject(activeRobot, LEGS[activeLeg].endEffector)
    if (!foot) return
    activeRobot.updateMatrixWorld(true)
    foot.getWorldPosition(targetPosition)
    targetRef.current.position.copy(targetPosition)
    const result = solveLegIK(activeRobot, activeLeg, targetPosition)
    publish(result, targetPosition)
  }

  const solveAtMarker = () => {
    if (!robot || !targetRef.current) return
    targetPosition.copy(targetRef.current.position)
    const result = solveLegIK(robot, activeLegRef.current, targetPosition)
    publish(result, targetPosition, draggingRef.current)
  }

  const captureGaitHomes = (activeRobot: URDFRobotLike) => {
    activeRobot.updateMatrixWorld(true)
    ;(Object.keys(LEGS) as LegId[]).forEach((legId) => {
      const foot = findObject(activeRobot, LEGS[legId].endEffector)
      if (!foot) return
      gaitHomesRef.current[legId] = foot.getWorldPosition(new THREE.Vector3())
    })
  }

  const resetPhysics = (activeRobot = robot) => {
    physicsRef.current = createPhysicsState()
    travelRef.current = 0
    const controls = controlsRef.current
    if (controls && cameraTravelRef.current !== 0) {
      controls.object.position.y += cameraTravelRef.current
      controls.target.y += cameraTravelRef.current
      controls.update()
    }
    cameraTravelRef.current = 0
    if (activeRobot) resetBodyTransform(activeRobot)
  }

  useEffect(() => {
    const manager = new THREE.LoadingManager()
    const loader = new URDFLoader(manager)
    loader.load(
      '/robodog/urdf/robodog.urdf',
      (loaded) => {
        const typedRobot = loaded as URDFRobotLike
        typedRobot.name = 'robodog-model'
        typedRobot.traverse((child) => {
          if (child instanceof THREE.Mesh) {
            child.castShadow = true
            child.receiveShadow = true
            child.material = Array.isArray(child.material)
              ? child.material.map((material) => material.clone())
              : child.material.clone()
            const materials = Array.isArray(child.material) ? child.material : [child.material]
            materials.forEach((material) => {
              if ('color' in material) (material as THREE.MeshPhongMaterial).color.set('#9aa8b5')
              if ('shininess' in material) (material as THREE.MeshPhongMaterial).shininess = 38
            })
          }
        })
        applyMinimumPose(typedRobot)
        setRobot(typedRobot)
      },
      undefined,
      (error) => setLoadError(error instanceof Error ? error.message : 'Could not load the URDF model.'),
    )
  }, [])

  useEffect(() => {
    activeLegRef.current = leg
    if (gait.mode === 'pose' || !gait.playing) {
      requestAnimationFrame(() => placeTargetAtFoot(robot, leg))
    }
  }, [leg, robot, gait.mode, gait.playing])

  useEffect(() => {
    if (gait.mode !== 'walk') return
    setSelectedPathPoint(-1)
    setSelectedPathObject(null)
    setPathRevision((revision) => revision + 1)
  }, [leg, gait.mode])

  useEffect(() => {
    if (!robot) return
    resetPhysics(robot)
    if (gait.mode === 'walk') applySpringPose(robot)
    else applyMinimumPose(robot)
    robot.updateMatrixWorld(true)
    captureGaitHomes(robot)
    gaitTimeRef.current = 0
    lastGaitPublishRef.current = 0
    requestAnimationFrame(() => placeTargetAtFoot(robot, activeLegRef.current))
  }, [robot, gait.mode])

  useEffect(() => {
    resetPhysics(robot)
    if (!robot || gait.mode !== 'walk') return
    captureGaitHomes(robot)
    ;(Object.keys(LEGS) as LegId[]).forEach((legId) => {
      const home = gaitHomesRef.current[legId]
      const marker = contactMarkerRefs.current[legId]
      if (!home || !marker) return
      marker.position.set(home.x, home.y, GROUND_HEIGHT + 0.004)
    })
    comMarkerRef.current?.position.copy(BODY_COM)
  }, [robot, gait.mode, gait.physics])

  useEffect(() => {
    if (!robot) return
    robot.traverse((child) => {
      if (!(child instanceof THREE.Mesh)) return
      const materials = Array.isArray(child.material) ? child.material : [child.material]
      materials.forEach((material) => {
        if ('color' in material) (material as THREE.MeshPhongMaterial).color.set('#7f8e9b')
        if ('emissive' in material) (material as THREE.MeshPhongMaterial).emissive.set('#000000')
      })
    })
    const selectedLinks = [
      `${leg}_top_leg_link`,
      `${leg}_mid_leg_link`,
      `${leg}_bot_leg_link`,
      LEGS[leg].endEffector,
    ]
    selectedLinks.forEach((name) => {
      findObject(robot, name)?.traverse((child) => {
        if (!(child instanceof THREE.Mesh)) return
        const materials = Array.isArray(child.material) ? child.material : [child.material]
        materials.forEach((material) => {
          if ('color' in material) (material as THREE.MeshPhongMaterial).color.set('#3f9fbd')
          if ('emissive' in material) (material as THREE.MeshPhongMaterial).emissive.set('#123541')
        })
      })
    })
  }, [leg, robot])

  useImperativeHandle(apiRef, () => ({
    setTarget: (next) => {
      if (!targetRef.current) return
      targetRef.current.position.set(...next)
      solveAtMarker()
    },
    homeTarget: () => placeTargetAtFoot(),
    resetPath: () => {
      pathOffsetsRef.current[activeLegRef.current] = createDefaultPath(activeLegRef.current)
      setSelectedPathPoint(-1)
      setSelectedPathObject(null)
      setPathRevision((revision) => revision + 1)
    },
    resetPose: () => {
      if (!robot) return
      resetPhysics(robot)
      if (gait.mode === 'walk') {
        applySpringPose(robot)
        captureGaitHomes(robot)
        gaitTimeRef.current = 0
      } else {
        applyMinimumPose(robot)
      }
      placeTargetAtFoot()
    },
    resetCamera: () => {
      const controls = controlsRef.current
      if (!controls) return
      const cameraFollow = Math.max(0, travelRef.current - 0.65)
      cameraTravelRef.current = cameraFollow
      controls.object.position.set(4.2, -4.8 - cameraFollow, 3.1)
      controls.target.set(0, -0.1 - cameraFollow, 0.38)
      controls.update()
    },
  }), [robot, gait.mode])

  useFrame((state, delta) => {
    if (!robot || gait.mode !== 'walk' || !gait.playing) return
    gaitTimeRef.current = (gaitTimeRef.current + delta * gait.speed) % 1
    const phase = gaitTimeRef.current
    const swingIndex = Math.floor(phase * 4) % GAIT_ORDER.length
    const swingLeg = GAIT_ORDER[swingIndex]
    const averageStride = GAIT_ORDER.reduce((total, legId) => {
      const path = pathOffsetsRef.current[legId]
      return total + Math.max(0.02, path[0].y - path[2].y)
    }, 0) / GAIT_ORDER.length
    const forwardSpeed = averageStride / (1 - SWING_FRACTION) * gait.speed
    travelRef.current += forwardSpeed * delta
    const distance = travelRef.current
    const cameraFollow = Math.max(0, distance - 0.65)
    const cameraStep = cameraFollow - cameraTravelRef.current
    if (controlsRef.current && cameraStep !== 0) {
      controlsRef.current.object.position.y -= cameraStep
      controlsRef.current.target.y -= cameraStep
      controlsRef.current.update()
      cameraTravelRef.current = cameraFollow
    }
    let contactCount = 0
    let selectedResult: IKResult | null = null

    GAIT_ORDER.forEach((legId, index) => {
      const startPhase = index / GAIT_ORDER.length
      const localPhase = (phase - startPhase + 1) % 1
      const inContact = localPhase >= SWING_FRACTION
      contactStatesRef.current[legId] = inContact
      if (inContact) contactCount += 1
    })

    if (gait.physics) {
      const physics = physicsRef.current
      const dt = Math.min(delta, 0.025)
      let totalForce = 0
      let rollTorque = 0
      let pitchTorque = 0

      GAIT_ORDER.forEach((legId) => {
        if (!contactStatesRef.current[legId]) return
        const mount = SUPPORT_MOUNTS[legId]
        const displacement = physics.z + physics.roll * mount.y - physics.pitch * mount.x
        const velocity = physics.vz + physics.rollVelocity * mount.y - physics.pitchVelocity * mount.x
        const force = Math.max(0, LEG_STIFFNESS * (STATIC_COMPRESSION - displacement) - LEG_DAMPING * velocity)
        totalForce += force
        rollTorque += mount.y * force
        pitchTorque -= mount.x * force
      })

      physics.vz += ((totalForce - PHYSICS_MASS * PHYSICS_GRAVITY) / PHYSICS_MASS) * dt
      physics.z = THREE.MathUtils.clamp(physics.z + physics.vz * dt, -0.12, 0.07)
      physics.rollVelocity += ((rollTorque / ANGULAR_INERTIA) - ANGULAR_DAMPING * physics.rollVelocity) * dt
      physics.pitchVelocity += ((pitchTorque / ANGULAR_INERTIA) - ANGULAR_DAMPING * physics.pitchVelocity) * dt
      physics.roll = THREE.MathUtils.clamp(physics.roll + physics.rollVelocity * dt, -0.14, 0.14)
      physics.pitch = THREE.MathUtils.clamp(physics.pitch + physics.pitchVelocity * dt, -0.14, 0.14)
      applyBodyTransform(robot, physics, distance)
    } else {
      applyBodyTransform(robot, physicsRef.current, distance)
    }

    GAIT_ORDER.forEach((legId, index) => {
      const home = gaitHomesRef.current[legId]
      if (!home) return
      const startPhase = index / GAIT_ORDER.length
      const localPhase = (phase - startPhase + 1) % 1
      sampleFootPath(pathOffsetsRef.current[legId], localPhase, pathSample)
      gaitTarget.copy(home).add(pathSample)
      gaitTarget.y -= distance
      const marker = contactMarkerRefs.current[legId]
      if (marker) {
        marker.position.set(gaitTarget.x, gaitTarget.y, GROUND_HEIGHT + 0.004)
        marker.scale.setScalar(contactStatesRef.current[legId] ? 1 : 0.72)
        marker.traverse((child) => {
          if (!(child instanceof THREE.Mesh)) return
          const material = child.material as THREE.MeshBasicMaterial
          material.color.set(contactStatesRef.current[legId] ? '#74d69a' : '#ff9f1c')
          material.opacity = contactStatesRef.current[legId] ? 0.92 : 0.28
        })
      }
      const result = solveLegIK(robot, legId, gaitTarget)
      if (legId === activeLegRef.current) {
        selectedResult = result
        targetPosition.copy(gaitTarget)
        targetRef.current?.position.copy(gaitTarget)
      }
    })

    if (comMarkerRef.current) {
      comWorldPosition.copy(BODY_COM).applyQuaternion(robot.quaternion).add(robot.position)
      comMarkerRef.current.position.copy(comWorldPosition)
    }
    if (pathEditorRef.current) {
      const pathHome = gaitHomesRef.current[activeLegRef.current]
      if (pathHome) pathEditorRef.current.position.set(pathHome.x, pathHome.y - distance, pathHome.z)
    }

    if (state.clock.elapsedTime - lastGaitPublishRef.current > 0.075 && selectedResult) {
      lastGaitPublishRef.current = state.clock.elapsedTime
      publish(selectedResult, targetPosition, true)
      const physics = physicsRef.current
      onGaitUpdate({
        phase,
        swingLeg,
        contacts: contactCount,
        bodyOffset: gait.physics ? physics.z : 0,
        roll: gait.physics ? THREE.MathUtils.radToDeg(physics.roll) : 0,
        pitch: gait.physics ? THREE.MathUtils.radToDeg(physics.pitch) : 0,
        distance,
        speed: forwardSpeed,
      })
    }
  })

  useEffect(() => {
    if (!loadError) return
    onTelemetry({
      converged: false,
      error: Infinity,
      iterations: 0,
      joints: [],
      actual: [0, 0, 0],
      target: [0, 0, 0],
      loaded: false,
      solving: false,
      message: loadError,
    })
  }, [loadError, onTelemetry])

  const selectedPathHome = gaitHomesRef.current[leg]

  return (
    <>
      <color attach="background" args={['#0b1015']} />
      <fog attach="fog" args={['#0b1015', 6, 12]} />
      <ambientLight intensity={0.42} />
      <directionalLight position={[4, -3, 7]} intensity={1.3} color="#e9f4ff" castShadow shadow-mapSize={[2048, 2048]} />
      <directionalLight position={[-4, 2, 2]} intensity={0.42} color="#4aa8ff" />
      <Grid
        args={[12, 12]}
        position={[0, 0, gait.mode === 'walk' ? GROUND_HEIGHT : -0.31]}
        rotation={[Math.PI / 2, 0, 0]}
        cellSize={0.25}
        cellThickness={0.45}
        cellColor="#2b3946"
        sectionSize={1}
        sectionThickness={0.9}
        sectionColor="#466077"
        fadeDistance={8}
        infiniteGrid
      />
      <axesHelper args={[0.5]} position={[-0.8, 1.5, 0.01]} />
      {gait.mode === 'walk' && gait.physics && (Object.keys(LEGS) as LegId[]).map((legId) => (
        <group
          key={`contact-${legId}`}
          ref={(group) => {
            if (group) contactMarkerRefs.current[legId] = group
            else delete contactMarkerRefs.current[legId]
          }}
        >
          <mesh>
            <ringGeometry args={[0.065, 0.105, 32]} />
            <meshBasicMaterial color="#74d69a" transparent opacity={0.92} side={THREE.DoubleSide} depthWrite={false} />
          </mesh>
          <mesh position={[0, 0, 0.007]}>
            <circleGeometry args={[0.025, 20]} />
            <meshBasicMaterial color="#74d69a" transparent opacity={0.92} side={THREE.DoubleSide} depthWrite={false} />
          </mesh>
        </group>
      ))}
      {gait.mode === 'walk' && gait.physics && (
        <group ref={comMarkerRef}>
          <mesh>
            <sphereGeometry args={[0.045, 18, 14]} />
            <meshBasicMaterial color="#ff9f1c" depthTest={false} />
          </mesh>
          <Line
            points={[[0, 0, -0.02], [0, 0, -0.95]]}
            color="#ff9f1c"
            lineWidth={1.2}
            transparent
            opacity={0.55}
            depthTest={false}
          />
        </group>
      )}
      {robot && <primitive object={robot} />}
      {robot && gait.mode === 'walk' && selectedPathHome && (
        <group ref={pathEditorRef} position={selectedPathHome.toArray()}>
          <Line
            points={visiblePathSamples}
            color="#ff9f1c"
            lineWidth={2.6}
            transparent
            opacity={0.92}
            depthTest={false}
            renderOrder={10}
          />
          {pathOffsetsRef.current[leg].map((point, index) => (
            <mesh
              key={`${leg}-${index}`}
              position={point.toArray()}
              renderOrder={11}
              onPointerDown={(event) => {
                event.stopPropagation()
                event.object.userData.pathLeg = leg
                setSelectedPathPoint(index)
                setSelectedPathObject(event.object as THREE.Mesh)
              }}
            >
              <sphereGeometry args={[index === selectedPathPoint ? 0.055 : 0.042, 18, 14]} />
              <meshStandardMaterial
                color={index === selectedPathPoint ? '#ff9f1c' : '#74c7ec'}
                emissive={index === selectedPathPoint ? '#7a3100' : '#123541'}
                emissiveIntensity={0.65}
                roughness={0.3}
                depthTest={false}
              />
            </mesh>
          ))}
        </group>
      )}
      <group ref={targetRef}>
        <TargetMarker />
      </group>
      {robot && targetRef.current && gait.mode === 'pose' && (
        <TransformControls
          object={targetRef.current}
          mode="translate"
          size={0.72}
          translationSnap={undefined}
          onMouseDown={() => {
            draggingRef.current = true
            if (controlsRef.current) controlsRef.current.enabled = false
          }}
          onObjectChange={solveAtMarker}
          onMouseUp={() => {
            draggingRef.current = false
            if (controlsRef.current) controlsRef.current.enabled = true
            solveAtMarker()
          }}
        />
      )}
      {robot && gait.mode === 'walk' && selectedPathObject?.userData.pathLeg === leg && (
        <TransformControls
          object={selectedPathObject}
          mode="translate"
          space="world"
          size={0.58}
          translationSnap={undefined}
          onMouseDown={() => {
            draggingRef.current = true
            if (controlsRef.current) controlsRef.current.enabled = false
          }}
          onObjectChange={() => {
            const pointObject = selectedPathObject
            if (!pointObject) return
            pathOffsetsRef.current[activeLegRef.current][selectedPathPoint].copy(pointObject.position)
            setPathRevision((revision) => revision + 1)

            if (!gait.playing) {
              const home = gaitHomesRef.current[activeLegRef.current]
              if (!home) return
              gaitTarget.copy(home).add(pointObject.position)
              const result = solveLegIK(robot, activeLegRef.current, gaitTarget)
              targetPosition.copy(gaitTarget)
              targetRef.current?.position.copy(gaitTarget)
              publish(result, gaitTarget, true)
            }
          }}
          onMouseUp={() => {
            draggingRef.current = false
            if (controlsRef.current) controlsRef.current.enabled = true
          }}
        />
      )}
      <CameraController controlsRef={controlsRef} />
    </>
  )
}

export const RobotViewport = forwardRef<ViewerHandle, {
  leg: LegId
  gait: GaitSettings
  onTelemetry: (value: ViewerTelemetry) => void
  onGaitUpdate: (value: GaitTelemetry) => void
}>(function RobotViewport({ leg, gait, onTelemetry, onGaitUpdate }, ref) {
  const apiRef = useRef<ViewerHandle | null>(null)
  useImperativeHandle(ref, () => ({
    setTarget: (target) => apiRef.current?.setTarget(target),
    homeTarget: () => apiRef.current?.homeTarget(),
    resetPath: () => apiRef.current?.resetPath(),
    resetPose: () => apiRef.current?.resetPose(),
    resetCamera: () => apiRef.current?.resetCamera(),
  }), [])

  return (
    <Canvas
      shadows="percentage"
      camera={{ position: [4.2, -4.8, 3.1], fov: 38, near: 0.05, far: 50, up: [0, 0, 1] }}
      dpr={[1, 2]}
      gl={{ antialias: true, toneMapping: THREE.ACESFilmicToneMapping, toneMappingExposure: 0.88 }}
    >
      <RobotScene leg={leg} gait={gait} onTelemetry={onTelemetry} onGaitUpdate={onGaitUpdate} apiRef={apiRef} />
    </Canvas>
  )
})
