export type RobotId = 'RMC1' | 'RMC2'

export type Point = { x: number; y: number }
export type Pose = Point & { yaw: number }

export type RobotState = {
  online: boolean
  lastMessageMs: number | null
  pose: Pose
  velocity: { x: number; y: number; yaw: number }
  battery: number | null
  lidar: Point[]
  trail: Point[]
  plan: Point[]
  goal: Pose | null
  emergency: boolean
  arucoId: string | null
  liftStatus: string | null
  gripper: 'open' | 'closed' | 'unknown'
}

export type MapState = {
  width: number
  height: number
  resolution: number
  origin: Point
  data: number[]
}

export type FmsState = {
  type: 'state'
  timestamp: number
  mode: 'ros' | 'mock'
  bridgeOnline: boolean
  bridgeError: string | null
  map: MapState | null
  robots: Record<RobotId, RobotState>
}

export type Command =
  | { type: 'manual'; robot: RobotId; x: number; y: number; yaw: number }
  | { type: 'stop'; robot: RobotId }
  | { type: 'emergency'; robot: RobotId; active: boolean }
  | { type: 'set_pose'; robot: RobotId; pose: Pose }
  | { type: 'set_goal'; robot: RobotId; pose: Pose }
  | { type: 'cancel_goal'; robot: RobotId }
  | { type: 'lift'; robot: 'RMC2'; height: 0 | 0.1 }
  | { type: 'gripper'; robot: 'RMC1'; state: 'open' | 'closed' }

