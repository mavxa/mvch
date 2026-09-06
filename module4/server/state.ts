import type { Command, FmsState, Point, RobotId, RobotState } from '../src/types'

const robot = (x: number, y: number): RobotState => ({
  online: false,
  lastMessageMs: null,
  pose: { x, y, yaw: 0 },
  velocity: { x: 0, y: 0, yaw: 0 },
  battery: null,
  lidar: [],
  trail: [{ x, y }],
  plan: [],
  goal: null,
  emergency: false,
  arucoId: null,
  liftStatus: null,
  gripper: 'unknown',
})

export function initialState(mode: 'ros' | 'mock'): FmsState {
  const mockMap = mode === 'mock' ? makeMockMap() : null
  return {
    type: 'state',
    timestamp: Date.now(),
    mode,
    bridgeOnline: mode === 'mock',
    bridgeError: null,
    map: mockMap,
    robots: { RMC1: robot(-2, 3), RMC2: robot(0, 0) },
  }
}

function makeMockMap() {
  const width = 60
  const height = 60
  const data = Array<number>(width * height).fill(0)
  for (let row = 0; row < height; row += 1) {
    for (let col = 0; col < width; col += 1) {
      const border = row < 2 || col < 2 || row > height - 3 || col > width - 3
      const shelf = col >= 7 && col <= 10 && row >= 8 && row <= 46
      if (border || shelf) data[row * width + col] = 100
    }
  }
  return { width, height, resolution: 0.1, origin: { x: -5.5, y: -0.5 }, data }
}

const clamp = (value: number, min: number, max: number) => Math.max(min, Math.min(max, value))

export function applyMockCommand(state: FmsState, command: Command): void {
  const rover = state.robots[command.robot]
  if (!rover) return

  if (command.type === 'emergency') {
    rover.emergency = command.active
    rover.velocity = { x: 0, y: 0, yaw: 0 }
  } else if (command.type === 'stop' || command.type === 'cancel_goal') {
    rover.velocity = { x: 0, y: 0, yaw: 0 }
    if (command.type === 'cancel_goal') {
      rover.goal = null
      rover.plan = []
    }
  } else if (command.type === 'manual' && !rover.emergency) {
    rover.velocity = {
      x: clamp(command.x, -0.7, 0.7),
      y: command.robot === 'RMC1' ? clamp(command.y, -0.7, 0.7) : 0,
      yaw: clamp(command.yaw, -1.2, 1.2),
    }
  } else if (command.type === 'set_pose') {
    rover.pose = command.pose
    rover.trail = [{ x: command.pose.x, y: command.pose.y }]
  } else if (command.type === 'set_goal') {
    rover.goal = command.pose
    rover.plan = [
      { x: rover.pose.x, y: rover.pose.y },
      { x: command.pose.x, y: command.pose.y },
    ]
  } else if (command.type === 'lift') {
    rover.liftStatus = command.height > 0 ? 'up' : 'down'
  } else if (command.type === 'gripper') {
    rover.gripper = command.state
  }
  state.timestamp = Date.now()
}

export function tickMock(state: FmsState, dt = 0.1): void {
  const now = Date.now()
  for (const id of ['RMC1', 'RMC2'] as RobotId[]) {
    const rover = state.robots[id]
    rover.online = true
    rover.lastMessageMs = 0
    rover.battery = id === 'RMC1' ? 24.4 : 23.9
    rover.pose.yaw += rover.velocity.yaw * dt
    const c = Math.cos(rover.pose.yaw)
    const s = Math.sin(rover.pose.yaw)
    rover.pose.x += (rover.velocity.x * c - rover.velocity.y * s) * dt
    rover.pose.y += (rover.velocity.x * s + rover.velocity.y * c) * dt
    if (rover.trail.length === 0 || Math.hypot(rover.pose.x - rover.trail.at(-1)!.x, rover.pose.y - rover.trail.at(-1)!.y) > 0.03) {
      rover.trail.push({ x: rover.pose.x, y: rover.pose.y })
      if (rover.trail.length > 180) rover.trail.shift()
    }
    const points: Point[] = []
    for (let angle = 0; angle < Math.PI * 2; angle += Math.PI / 28) {
      const distance = 1.0 + 0.35 * Math.sin(angle * 3 + now / 1200)
      points.push({
        x: rover.pose.x + Math.cos(angle + rover.pose.yaw) * distance,
        y: rover.pose.y + Math.sin(angle + rover.pose.yaw) * distance,
      })
    }
    rover.lidar = points
    if (id === 'RMC2') {
      const row = Math.max(0, Math.min(5, Math.round(-rover.pose.x)))
      const col = Math.max(0, Math.min(5, Math.round(rover.pose.y)))
      rover.arucoId = String(row * 6 + col)
    }
  }
  state.timestamp = now
}
