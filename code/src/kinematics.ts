import * as THREE from 'three'

export type LegId = 'front_left' | 'front_right' | 'back_left' | 'back_right'

export type JointReading = {
  name: string
  shortName: string
  radians: number
  degrees: number
  lower: number
  upper: number
}

export type IKResult = {
  converged: boolean
  error: number
  iterations: number
  joints: JointReading[]
  actual: [number, number, number]
}

export type URDFJointLike = THREE.Object3D & {
  axis?: THREE.Vector3
  limit?: { lower?: number; upper?: number }
  jointType?: string
  angle?: number
  setJointValue: (value: number) => void
}

export type URDFRobotLike = THREE.Object3D & {
  joints: Record<string, URDFJointLike>
}

export const SPRING_STANCE_DELTA = {
  longitudinal: 0.3374,
  vertical: 0.3584,
}

export const LEGS: Record<LegId, {
  label: string
  code: string
  joints: [string, string, string]
  endEffector: string
}> = {
  front_left: {
    label: 'Front left',
    code: 'FL',
    joints: ['front_left_top_leg_link_joint', 'front_left_mid_leg_link_joint', 'front_left_bot_leg_link_joint'],
    endEffector: 'front_left_leg_foot_link',
  },
  front_right: {
    label: 'Front right',
    code: 'FR',
    joints: ['front_right_top_leg_link_joint', 'front_right_mid_leg_link_joint', 'front_right_bot_leg_link_joint'],
    endEffector: 'front_right_leg_foot_link',
  },
  back_left: {
    label: 'Back left',
    code: 'BL',
    joints: ['back_left_top_leg_link_joint', 'back_left_mid_leg_link_joint', 'back_left_bot_leg_link_joint'],
    endEffector: 'back_left_leg_foot_link',
  },
  back_right: {
    label: 'Back right',
    code: 'BR',
    joints: ['back_right_top_leg_link_joint', 'back_right_mid_leg_link_joint', 'back_right_bot_leg_link_joint'],
    endEffector: 'back_right_leg_foot_link',
  },
}

const FALLBACK_LIMITS: Record<string, [number, number]> = {
  back_left_top_leg_link_joint: [0, 2.0944],
  back_left_mid_leg_link_joint: [0, 2.41],
  back_left_bot_leg_link_joint: [0, 3.83973],
  back_right_top_leg_link_joint: [0, 2.0944],
  back_right_mid_leg_link_joint: [0, 2.7508],
  back_right_bot_leg_link_joint: [0, 3.83973],
  front_left_top_leg_link_joint: [0, 2.0944],
  front_left_mid_leg_link_joint: [0, 2.6708],
  front_left_bot_leg_link_joint: [0, 3.89253],
  front_right_top_leg_link_joint: [0, 2.0944],
  front_right_mid_leg_link_joint: [0, 2.6708],
  front_right_bot_leg_link_joint: [0, 3.89253],
}

function getLimits(joint: URDFJointLike): [number, number] {
  const fallback = FALLBACK_LIMITS[joint.name] ?? [0, Math.PI * 2]
  return [joint.limit?.lower ?? fallback[0], joint.limit?.upper ?? fallback[1]]
}

function currentAngle(joint: URDFJointLike) {
  return typeof joint.angle === 'number' ? joint.angle : 0
}

export function setJointAngle(joint: URDFJointLike, value: number) {
  const [lower, upper] = getLimits(joint)
  joint.setJointValue(THREE.MathUtils.clamp(value, lower, upper))
}

export function findObject(robot: THREE.Object3D, name: string) {
  return robot.getObjectByName(name) ?? null
}

export function solveLegIK(
  robot: URDFRobotLike,
  legId: LegId,
  target: THREE.Vector3,
  tolerance = 0.004,
): IKResult {
  const definition = LEGS[legId]
  const joints = definition.joints.map((name) => robot.joints[name]).filter(Boolean)
  const endEffector = findObject(robot, definition.endEffector)

  if (joints.length !== 3 || !endEffector) {
    return { converged: false, error: Number.POSITIVE_INFINITY, iterations: 0, joints: [], actual: [0, 0, 0] }
  }

  const current = new THREE.Vector3()
  const jointPosition = new THREE.Vector3()
  const axis = new THREE.Vector3()
  const errorVector = new THREE.Vector3()
  const columns = joints.map(() => new THREE.Vector3())
  const inverse = new THREE.Matrix3()
  const system = new THREE.Matrix3()
  const correction = new THREE.Vector3()
  let error = Infinity
  let iterations = 0

  for (iterations = 0; iterations < 48; iterations += 1) {
    robot.updateMatrixWorld(true)
    endEffector.getWorldPosition(current)
    errorVector.copy(target).sub(current)
    error = errorVector.length()
    if (error < tolerance) break

    joints.forEach((joint, index) => {
      joint.getWorldPosition(jointPosition)
      axis.copy(joint.axis ?? new THREE.Vector3(0, 0, 1)).transformDirection(joint.matrixWorld)
      columns[index].copy(current).sub(jointPosition).crossVectors(axis, current.clone().sub(jointPosition))
    })

    const damping = 0.055
    let a00 = damping * damping
    let a01 = 0
    let a02 = 0
    let a11 = damping * damping
    let a12 = 0
    let a22 = damping * damping
    columns.forEach((column) => {
      a00 += column.x * column.x
      a01 += column.x * column.y
      a02 += column.x * column.z
      a11 += column.y * column.y
      a12 += column.y * column.z
      a22 += column.z * column.z
    })
    system.set(a00, a01, a02, a01, a11, a12, a02, a12, a22)
    inverse.copy(system).invert()
    correction.copy(errorVector).applyMatrix3(inverse)

    joints.forEach((joint, index) => {
      const rawStep = columns[index].dot(correction) * 0.82
      const step = THREE.MathUtils.clamp(rawStep, -0.14, 0.14)
      setJointAngle(joint, currentAngle(joint) + step)
    })
  }

  robot.updateMatrixWorld(true)
  endEffector.getWorldPosition(current)
  error = current.distanceTo(target)

  return {
    converged: error < tolerance,
    error,
    iterations,
    joints: joints.map((joint, index) => {
      const [lower, upper] = getLimits(joint)
      const radians = currentAngle(joint)
      return {
        name: joint.name,
        shortName: ['Hip', 'Thigh', 'Knee'][index],
        radians,
        degrees: THREE.MathUtils.radToDeg(radians),
        lower,
        upper,
      }
    }),
    actual: [current.x, current.y, current.z],
  }
}

export function applyStandingPose(robot: URDFRobotLike) {
  // Zero-based motor coordinates: q_new = q_old - old_lower_limit.
  const valuesByLeg: Record<LegId, [number, number, number]> = {
    front_left: [0.5236, 1.0308, 2.18],
    front_right: [1.5708, 1.64, 1.71253],
    back_left: [0.5236, 0.69, 2.1272],
    back_right: [1.5708, 1.72, 1.71253],
  }
  ;(Object.keys(LEGS) as LegId[]).forEach((legId) => {
    const values = valuesByLeg[legId]
    LEGS[legId].joints.forEach((name, index) => {
      const joint = robot.joints[name]
      if (joint) setJointAngle(joint, values[index])
    })
  })
  robot.updateMatrixWorld(true)
}

export function applyMinimumPose(robot: URDFRobotLike) {
  ;(Object.keys(LEGS) as LegId[]).forEach((legId) => {
    LEGS[legId].joints.forEach((name) => {
      const joint = robot.joints[name]
      if (joint) setJointAngle(joint, 0)
    })
  })
  robot.updateMatrixWorld(true)
}

export function applySpringPose(robot: URDFRobotLike) {
  applyStandingPose(robot)
  const legIds = Object.keys(LEGS) as LegId[]
  const starts = new Map<LegId, THREE.Vector3>()
  legIds.forEach((legId) => {
    const foot = findObject(robot, LEGS[legId].endEffector)
    if (foot) starts.set(legId, foot.getWorldPosition(new THREE.Vector3()))
  })

  // Approach the crouch in small steps so each differently-shaped chain
  // settles onto its own IK solution instead of sharing copied joint angles.
  for (let step = 1; step <= 12; step += 1) {
    const amount = step / 12
    legIds.forEach((legId) => {
      const start = starts.get(legId)
      if (!start) return
      const longitudinalDirection = legId.startsWith('front') ? -1 : 1
      const target = start.clone()
      target.y += SPRING_STANCE_DELTA.longitudinal * longitudinalDirection * amount
      target.z += SPRING_STANCE_DELTA.vertical * amount
      solveLegIK(robot, legId, target)
    })
  }
  robot.updateMatrixWorld(true)
}
