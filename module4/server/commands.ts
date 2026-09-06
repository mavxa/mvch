import type { Command, Pose, RobotId } from '../src/types'

const ROBOTS: RobotId[] = ['RMC1', 'RMC2']

function isRobot(value: unknown): value is RobotId {
  return typeof value === 'string' && ROBOTS.includes(value as RobotId)
}

function isFiniteNumber(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value)
}

function isPose(value: unknown): value is Pose {
  if (!value || typeof value !== 'object') return false
  const pose = value as Record<string, unknown>
  return isFiniteNumber(pose.x) && isFiniteNumber(pose.y) && isFiniteNumber(pose.yaw)
}

export function isCommand(value: unknown): value is Command {
  if (!value || typeof value !== 'object') return false
  const command = value as Record<string, unknown>
  if (!isRobot(command.robot) || typeof command.type !== 'string') return false

  switch (command.type) {
    case 'manual':
      return isFiniteNumber(command.x) && isFiniteNumber(command.y) && isFiniteNumber(command.yaw)
    case 'stop':
    case 'cancel_goal':
      return true
    case 'emergency':
      return typeof command.active === 'boolean'
    case 'set_pose':
    case 'set_goal':
      return isPose(command.pose)
    case 'lift':
      return command.robot === 'RMC2' && (command.height === 0 || command.height === 0.1)
    case 'gripper':
      return command.robot === 'RMC1' && (command.state === 'open' || command.state === 'closed')
    default:
      return false
  }
}
